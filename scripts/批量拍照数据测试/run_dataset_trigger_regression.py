#!/usr/bin/env python3
"""批量控制器：等价执行“按 Enter 准备 input → TCP 触发 → 记录 → 下一项”。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

from prepare_dataset_inputs import DATA_ROOT, INPUT_DIR, TestItem, parse_folder, prepare


PROJECT_ROOT = Path(__file__).resolve().parents[2]
TRIGGER = PROJECT_ROOT / "build/OpenmindTrajectoryTrigger"
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 9108



def parse_reply(output: str, tag: str) -> dict:
    """取触发器输出中 "[客户端][应答N] {json}" 那一行的 JSON；没有则返回空字典。"""
    for line in output.splitlines():
        if line.startswith(tag):
            try:
                return json.loads(line[len(tag):].strip())
            except json.JSONDecodeError:
                return {}
    return {}


def summarize(output: str) -> dict:
    """从触发器输出与本轮归档 manifest 中提取等级、失败原因与耗时。"""
    final = parse_reply(output, "[客户端][应答2]")
    archived = parse_reply(output, "[客户端][应答3]")
    summary = {"status": final.get("status", ""), "ready": bool(final.get("ready", False)),
               "grade": "-", "photo_to_grasp_grade": "-", "grasp_to_place_grade": "-",
               "candidate": -1, "reason": final.get("reason", ""),
               "failed_stage": final.get("failed_stage", ""),
               "archive_path": archived.get("archive_path", ""), "executable_ms": None}
    for line in output.splitlines():
        if line.startswith("[客户端][完成] 到可执行="):
            try:
                summary["executable_ms"] = float(line.split("到可执行=")[1].split("ms")[0])
            except (IndexError, ValueError):
                pass
    manifest_path = Path(summary["archive_path"]) / "output/trajectory_planning/trajectory_manifest.json"
    if summary["archive_path"] and manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            summary["grade"] = manifest.get("grade", "-")
            summary["photo_to_grasp_grade"] = manifest.get("photo_to_grasp_grade", "-")
            summary["grasp_to_place_grade"] = manifest.get("grasp_to_place_grade", "-")
            summary["candidate"] = manifest.get("selected_candidate_index", -1)
            if not summary["reason"]:
                summary["reason"] = manifest.get("reason", "")
            summary["stop_reason"] = (manifest.get("candidate_statistics") or {}).get("stop_reason", "")
        except (OSError, json.JSONDecodeError):
            pass
    return summary

def collect_items() -> list[TestItem]:
    items: list[TestItem] = []
    for folder in sorted(path for path in DATA_ROOT.iterdir() if path.is_dir()):
        try:
            items.extend(parse_folder(folder))
        except ValueError as error:
            # 与交互准备脚本一致：不完整拍照文件夹不进入待测清单。
            print(f"跳过不完整文件夹 {folder.name}: {error}", file=sys.stderr)
    return items


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-sequence", type=int, default=1,
                        help="从第几个测试项开始（1 起）；默认 1")
    parser.add_argument("--end-sequence", type=int, default=None,
                        help="执行至第几个测试项（含）；默认至最后一项")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--timeout-s", type=int, default=600)
    parser.add_argument("--interval-s", type=float, default=5.0,
                        help="每轮触发完成后、准备下一项前的等待秒数；默认 5")
    parser.add_argument("--workpiece-type", default=None,
                        help="仅测试指定工件号，例如 QR0010；默认测试全部工件")
    parser.add_argument("--retry-failures-from", type=Path, default=None,
                        help="仅重测指定 JSONL 报告中失败的文件夹/Workpiece[n] 条目")
    args = parser.parse_args()

    if not TRIGGER.is_file():
        print(f"错误：找不到触发器: {TRIGGER}", file=sys.stderr)
        return 2
    items = collect_items()
    if args.workpiece_type is not None:
        requested_type = args.workpiece_type.strip().upper()
        items = [item for item in items if item.workpiece_type == requested_type]
        if not items:
            print(f"错误：未找到工件号为 {requested_type} 的有效测试项", file=sys.stderr)
            return 2
    if args.retry_failures_from is not None:
        source_report = args.retry_failures_from
        if not source_report.is_file():
            print(f"错误：失败报告不存在: {source_report}", file=sys.stderr)
            return 2
        try:
            failed_keys = {
                (record["folder"], int(record["workpiece_index"]))
                for line in source_report.read_text(encoding="utf-8").splitlines()
                if line.strip()
                for record in [json.loads(line)]
                if not record.get("success", False)
            }
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as error:
            print(f"错误：无法解析失败报告 {source_report}: {error}", file=sys.stderr)
            return 2
        items = [item for item in items if (item.folder.name, item.workpiece_index) in failed_keys]
        if not items:
            print(f"错误：失败报告中没有与当前数据集匹配的失败测试项: {source_report}", file=sys.stderr)
            return 2
    if args.start_sequence < 1 or args.start_sequence > len(items):
        print(f"错误：--start-sequence 必须位于 1..{len(items)}", file=sys.stderr)
        return 2
    end_sequence = len(items) if args.end_sequence is None else args.end_sequence
    if end_sequence < args.start_sequence or end_sequence > len(items):
        print(f"错误：--end-sequence 必须位于 {args.start_sequence}..{len(items)}", file=sys.stderr)
        return 2

    report_dir = PROJECT_ROOT / "logs"
    report_dir.mkdir(parents=True, exist_ok=True)
    report_path = report_dir / f"dataset_trigger_regression_{datetime.now():%Y%m%d_%H%M%S}.jsonl"
    remaining = items[args.start_sequence - 1:end_sequence]
    success = 0
    failed = 0
    grades: Counter = Counter()
    reasons: Counter = Counter()
    print(f"批量测试开始：总项={len(items)}，本次范围={args.start_sequence}..{end_sequence}，待执行={len(remaining)}")
    print(f"报告: {report_path}")
    print("不启动/停止服务；每轮只准备 input，再调用既有 OpenmindTrajectoryTrigger。", flush=True)

    with report_path.open("w", encoding="utf-8") as report:
        for sequence, item in enumerate(remaining, start=args.start_sequence):
            record: dict[str, object] = {
                "sequence": sequence,
                "folder": item.folder.name,
                "workpiece_type": item.workpiece_type,
                "workpiece_index": item.workpiece_index,
                "workpiece_count_in_folder": item.workpiece_count,
            }
            try:
                # 等价于用户在 prepare_dataset_inputs.py 中按一次 Enter。
                prepare(item)
                command = [str(TRIGGER), "--input-dir", str(INPUT_DIR), "--host", args.host,
                           "--port", str(args.port), "--timeout-s", str(args.timeout_s)]
                result = subprocess.run(command, cwd=PROJECT_ROOT, text=True,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        timeout=args.timeout_s + 30, check=False)
                output = result.stdout
                info = summarize(output)
                ok = result.returncode == 0 and info["status"] == "completed" and info["ready"]
                record.update({"returncode": result.returncode, "success": ok, **info,
                               "trigger_output": output})
                if ok:
                    success += 1
                    state = "成功"
                else:
                    failed += 1
                    state = "失败"
            except subprocess.TimeoutExpired as error:
                failed += 1
                record.update({"success": False, "reason": f"触发器超时: {error}"})
                state = "失败/超时"
            except Exception as error:  # 单项异常必须记录并继续下一项。
                failed += 1
                record.update({"success": False, "reason": f"异常: {error}"})
                state = "失败/异常"
            if record.get("success"):
                grades[str(record.get("grade", "-"))] += 1
                detail = (f"等级={record.get('grade')}（抓取 {record.get('photo_to_grasp_grade')}"
                          f" / 放置 {record.get('grasp_to_place_grade')}）"
                          f" candidate={record.get('candidate')}")
            else:
                reason = str(record.get("reason") or record.get("failed_stage") or "未知")
                reasons[reason] += 1
                detail = f"原因={reason}"
            if record.get("executable_ms") is not None:
                detail += f" 可执行耗时={record['executable_ms']:.0f}ms"

            report.write(json.dumps(record, ensure_ascii=False) + "\n")
            report.flush()
            print(f"[{sequence}/{len(items)}] {state} {item.folder.name} / "
                  f"Workpiece[{item.workpiece_index}] | {detail} | success={success} failed={failed}",
                  flush=True)
            if sequence < end_sequence and args.interval_s > 0:
                time.sleep(args.interval_s)

    print(f"批量测试完成：总执行={len(remaining)} 成功={success} 失败={failed} 报告={report_path}")
    if grades:
        print("  等级分布: " + "  ".join(f"{grade}={count}" for grade, count in sorted(grades.items())))
    if reasons:
        print("  失败原因:")
        for reason, count in reasons.most_common():
            print(f"    {count:4d}  {reason}")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
