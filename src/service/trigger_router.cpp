/**
 * @file trigger_router.cpp
 * @brief 触发路由实现：单任务生命周期、manifest 三阶段发布与逐轮归档。
 */

#include "trajectory_plan/service/trigger_router.h"

#include <chrono>
#include <png.h>

#include "trajectory_plan/algorithm/trajectory_output_format_algorithm.h"
#include "trajectory_plan/pipeline/pipeline_context.h"
#include "trajectory_plan/service/output_publisher.h"
#include "trajectory_plan/service/response_builder.h"
#include "trajectory_plan/tools/json_util.h"
#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

/**
 * @brief 解析 request.txt 原文，取得本轮工件类型与位姿。
 *
 * 文字中的图片路径仅供客户端定位文件，service 不按这些路径读本地图片；
 * 计算使用报文里按主／左／右顺序绑定的内存数据。
 */
bool ParseRequest(const std::string& request_text, PipelineContext& ctx, std::string& error)
{
    std::string main_rel;
    std::string left_rel;
    std::string right_rel;
    std::string rgb_main_rel;
    if (!ParseRequestText(request_text, ctx.workpiece_type, ctx.t_b_o, ctx.t_base_flange,
                          ctx.t_base_flange_joint_angles_deg, main_rel, left_rel, right_rel,
                          rgb_main_rel, error))
    {
        return false;
    }
    if (ctx.workpiece_type.empty())
    {
        error = "request.txt 缺少 workpiece_type";
        return false;
    }
    // 刚体校验允许合理的小数舍入误差；缩放或剪切直接拒绝。
    if (!ValidateTransform(ctx.t_b_o, "T_B_O_target", error))
    {
        return false;
    }
    ctx.has_t_b_o = true;
    return true;
}

/** @brief 解码主相机 PNG；只在轨迹已回复客户端后调用，绝不进入规划关键路径。 */
bool DecodeRgbPng(const std::vector<uint8_t>& encoded, std::vector<uint8_t>& rgb,
                  uint32_t& width, uint32_t& height, std::string& error)
{
    rgb.clear();
    width = 0;
    height = 0;
    if (encoded.empty())
    {
        error = "主相机 RGB 图为空";
        return false;
    }
    png_image image {};
    image.version = PNG_IMAGE_VERSION;
    if (!png_image_begin_read_from_memory(&image, encoded.data(), encoded.size()))
    {
        error = image.message;
        return false;
    }
    image.format = PNG_FORMAT_RGB;
    rgb.resize(PNG_IMAGE_SIZE(image));
    if (!png_image_finish_read(&image, nullptr, rgb.data(), 0, nullptr))
    {
        error = image.message;
        png_image_free(&image);
        rgb.clear();
        return false;
    }
    width = image.width;
    height = image.height;
    png_image_free(&image);
    return true;
}

/** @brief 主相机点取真实 RGB；左右相机保留低亮中性灰，便于分辨工件。 */
std::vector<std::array<uint8_t, 3>> BuildVisualizationColors(const PipelineContext& ctx)
{
    std::vector<std::array<uint8_t, 3>> colors(ctx.cloud_b.size(), {105, 105, 105});
    std::vector<uint8_t> rgb;
    uint32_t width = 0;
    uint32_t height = 0;
    std::string error;
    if (!DecodeRgbPng(ctx.rgb_main_bytes, rgb, width, height, error))
    {
        LOG_WARN << "[后台可视化] RGB 解码失败，主点云使用灰色: " << error;
        return colors;
    }
    if (width != static_cast<uint32_t>(ctx.main_depth_width) ||
        height != static_cast<uint32_t>(ctx.main_depth_height))
    {
        LOG_WARN << "[后台可视化] RGB 尺寸=" << width << "x" << height
                 << " 与主深度=" << ctx.main_depth_width << "x" << ctx.main_depth_height
                 << " 不一致，主点云使用灰色";
        return colors;
    }
    const size_t pixel_count = static_cast<size_t>(width) * height;
    size_t colored = 0;
    for (size_t i = 0; i < colors.size() && i < ctx.cloud_camera_indices.size() &&
                       i < ctx.cloud_source_pixel_indices.size(); ++i)
    {
        if (ctx.cloud_camera_indices[i] != 0)
        {
            continue;
        }
        const size_t pixel = ctx.cloud_source_pixel_indices[i];
        if (pixel >= pixel_count)
        {
            continue;
        }
        const size_t offset = pixel * 3;
        colors[i] = {rgb[offset], rgb[offset + 1], rgb[offset + 2]};
        ++colored;
    }
    LOG_INFO << "[后台可视化] 主RGB=" << width << "x" << height
             << " 主相机上色点=" << colored << " 总点=" << colors.size();
    return colors;
}

} // namespace

bool TriggerRouter::Init(Pipeline* pipeline,
                         SessionArchive* archive,
                         const std::string& output_root,
                         const std::string& grasp_output_dir,
                         const std::string& trajectory_output_dir,
                         const std::string& collision_model_id,
                         const std::string& collision_model_sha256,
                         std::string& error)
{
    if (pipeline == nullptr || archive == nullptr || output_root.empty() ||
        grasp_output_dir.empty() || trajectory_output_dir.empty())
    {
        error = "TriggerRouter 依赖不完整";
        return false;
    }
    pipeline_ = pipeline;
    archive_ = archive;
    output_root_ = output_root;
    grasp_output_dir_ = grasp_output_dir;
    trajectory_output_dir_ = trajectory_output_dir;
    collision_model_id_ = collision_model_id;
    collision_model_sha256_ = collision_model_sha256;
    return true;
}

void TriggerRouter::SetServiceReady(bool ready)
{
    service_ready_ = ready;
}

void TriggerRouter::HandleFrame(const TriggerFrame& frame, const ResponseSender& sender)
{
    if (!service_ready_)
    {
        LOG_WARN << "[tcp][拒绝] 连接=" << frame.connection_id << " 原因=服务尚未就绪";
        sender(ResponseBuilder::Rejected("", "服务尚未就绪，初始化或预热未完成"));
        return;
    }
    // 单任务、不排队：占用中的新连接立即返回 busy，不创建新代次、不改动 output。
    std::unique_lock<std::mutex> lock(task_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
    {
        LOG_WARN << "[tcp][busy] 连接=" << frame.connection_id
                 << " 原因=已有任务在执行或归档中";
        sender(ResponseBuilder::Busy("已有任务在执行或归档中，本次请求未被接受"));
        return;
    }
    RunTask(frame, sender, nullptr);
}

bool TriggerRouter::RunOnce(const TriggerFrame& frame, std::string& reply)
{
    std::lock_guard<std::mutex> lock(task_mutex_);
    const ResponseSender sink = [](const std::string&) { return true; };
    RunTask(frame, sink, &reply);
    return reply.find("\"status\":\"completed\"") != std::string::npos;
}

void TriggerRouter::RunTask(const TriggerFrame& frame, const ResponseSender& sender,
                            std::string* reply_out)
{
    const auto task_started = std::chrono::steady_clock::now();

    PipelineContext ctx;
    ctx.connection_id = frame.connection_id;
    ctx.request_text = frame.request_text;
    ctx.depth_main_bytes = frame.depth_main;
    ctx.depth_left_bytes = frame.depth_left;
    ctx.depth_right_bytes = frame.depth_right;
    ctx.rgb_main_bytes = frame.rgb_main;
    ctx.grasp_output_dir = grasp_output_dir_;
    ctx.trajectory_output_dir = trajectory_output_dir_;
    ctx.collision_model_id = collision_model_id_;
    ctx.collision_model_sha256 = collision_model_sha256_;

    // ---- 输入校验：失败即 rejected，不创建代次、不改动 output ----
    std::string error;
    if (!ParseRequest(frame.request_text, ctx, error))
    {
        LOG_WARN << "[tcp][拒绝] 连接=" << frame.connection_id << " 原因=" << error;
        const std::string reply = ResponseBuilder::Rejected("", error);
        sender(reply);
        if (reply_out != nullptr)
        {
            *reply_out = reply;
        }
        return;
    }

    ctx.request_id = "req_" + TimestampMilliseconds() + "_" + std::to_string(++trigger_sequence_);
    ctx.generation_id = RandomHex32();
    LOG_INFO << "[tcp][接收3/3] 请求校验通过 连接=" << frame.connection_id
             << " request_id=" << ctx.request_id << " generation_id=" << ctx.generation_id
             << " 工件=" << ctx.workpiece_type;
    LOG_INFO << "[pipeline][trigger] request_id=" << ctx.request_id
             << " generation_id=" << ctx.generation_id;
    LOG_INFO << "[pipeline][input_validation] result=pass depth_images=3 rgb_images=1 文字字节="
             << frame.request_text.size() << " 图片字节=" << frame.depth_main.size() << "/"
             << frame.depth_left.size() << "/" << frame.depth_right.size() << "/"
             << frame.rgb_main.size();

    // ---- 先原子发布处理中 manifest，使上轮结果失效；失败则不接受任务 ----
    if (!OutputPublisher::PublishProcessing(trajectory_output_dir_, ctx.request_id,
                                            ctx.generation_id, ctx.workpiece_type, error))
    {
        LOG_ERROR << "[pipeline][触发] 处理中状态发布失败，拒绝任务: " << error;
        const std::string reply =
            ResponseBuilder::Rejected(ctx.request_id, "本轮状态文件写入失败: " + error);
        sender(reply);
        if (reply_out != nullptr)
        {
            *reply_out = reply;
        }
        return;
    }
    LOG_INFO << "[pipeline][触发] output状态=处理中 ready=false 应答=accepted";
    // accepted 仅表示任务获准执行，不表示规划成功。
    sender(ResponseBuilder::Accepted(ctx.request_id, ctx.generation_id));

    // ---- 建本轮归档目录并立即保存收到的原始文字、三张深度图与主 RGB 图 ----
    TriggerArchive archive = archive_->BeginTrigger(frame.connection_id);
    ArchiveStatus archive_status;
    archive_status.path = archive.directory;
    std::string archive_error;
    if (archive.opened)
    {
        if (!SessionArchive::SaveInputs(archive, frame.request_text, frame.depth_main,
                                        frame.depth_left, frame.depth_right, frame.rgb_main, archive_error))
        {
            archive_status.status = "failed";
            archive_status.error = archive_error;
        }
    }
    else
    {
        archive_status.status = "failed";
        archive_status.error = archive.open_error;
    }

    // ---- 执行两条 flow ----
    std::string failed_flow;
    std::string failed_stage;
    std::string failure_reason;
    bool planning_ok = true;
    for (const std::string& flow : pipeline_->EnabledFlows())
    {
        FlowResult result;
        if (!pipeline_->Run(flow, ctx, result))
        {
            planning_ok = false;
            failed_flow = flow;
            failed_stage = result.failed_stage;
            failure_reason = result.reason;
            LOG_ERROR << "[pipeline][flow衔接] " << flow << "=失败，后续 flow 跳过";
            break;
        }
        LOG_INFO << "[pipeline][flow衔接] " << flow << "=成功";
    }

    // 正常搜索无解：flow 全部执行完成，但本轮没有可用轨迹。输出 stage 已把
    // manifest 写成 failed/ready=false；最终应答同样按 failed 报告，使客户端
    // 不会把"没找到轨迹"误当成有结果可执行。
    if (planning_ok && !ctx.has_solution)
    {
        failed_flow = "trajectory";
        failed_stage = "plan_full_cycle";
        failure_reason = ctx.failure_reason.empty() ? "未找到满足要求的完整周期" : ctx.failure_reason;
    }

    // ---- 异常失败收尾：统一发布 ready=false 的 manifest 并说明原因 ----
    if (!planning_ok)
    {
        // 先清掉不属于本轮的受管产物，避免上轮成功轨迹被归档成本轮结果。
        std::vector<std::string> purged;
        std::string purge_error;
        if (OutputPublisher::PurgeForeignGeneration(trajectory_output_dir_, grasp_output_dir_,
                                                    ctx.generation_id, !ctx.cloud_b.empty(), purged,
                                                    purge_error))
        {
            std::string purged_list;
            for (size_t index = 0; index < purged.size(); ++index)
            {
                purged_list += (index > 0 ? ", " : "") + purged[index];
            }
            LOG_INFO << "[pipeline][异常收尾] 旧代次产物清理=完成 删除=" << purged.size()
                     << "个 受管文件=[" << purged_list << "]";
        }
        else
        {
            LOG_ERROR << "[pipeline][异常收尾] 旧代次产物清理失败: " << purge_error;
        }
        std::string publish_error;
        const bool published = OutputPublisher::PublishFailed(
            trajectory_output_dir_, ctx.request_id, ctx.generation_id, ctx.workpiece_type,
            failed_flow, failed_stage, failure_reason, publish_error);
        LOG_ERROR << "[pipeline][异常收尾] generation_id=" << ctx.generation_id
                  << " ready=false 状态文件写入=" << (published ? "成功" : ("失败: " + publish_error));
    }

    // ================= 改动六：轨迹优先应答 =================
    // 到此为止轨迹与 manifest 已发布，客户端拿到第二行即可驱动机械臂。
    // 点云只供第三流程可视化、归档只供事后排查，都不该让客户端等。
    const double reply_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - task_started)
            .count();

    // ---- 第二行：completed / failed ----
    const bool completed = planning_ok && ctx.has_solution;
    std::string reply;
    if (completed)
    {
        reply = ResponseBuilder::Completed(ctx.request_id, ctx.generation_id, true,
                                           OutputPublisher::ManifestPath(trajectory_output_dir_));
    }
    else
    {
        reply = ResponseBuilder::Failed(ctx.request_id, ctx.generation_id, failed_flow, failed_stage,
                                        failure_reason);
    }
    const bool sent = sender(reply);
    LOG_INFO << "[tcp][轨迹应答] " << (completed ? "completed" : "failed")
             << " 到客户端可执行耗时=" << reply_ms << "ms 发送=" << (sent ? "成功" : "失败");
    if (reply_out != nullptr)
    {
        *reply_out = reply;
    }

    // ================= 以下在后台收尾，不计入触发耗时 =================
    // 仍在同一个 task_mutex_ 作用域内，因此收尾期间新连接照常返回 busy，
    // 单任务占用持续到第三行 archived 发出为止。

    // ---- 后台 1：写点云并补发同代次 manifest ----
    bool point_cloud_ready = false;
    {
        const std::vector<std::array<uint8_t, 3>> colors = BuildVisualizationColors(ctx);
        const std::string ply = TrajectoryOutputFormatAlgorithm::BuildPlyBinary(
            ctx.cloud_b, colors, {"coordinate_frame base_link", "unit mm"});
        const std::string cloud_path =
            JoinPath(trajectory_output_dir_, "point_cloud_B.ply");
        std::string cloud_error;
        if (!WriteTextFileAtomic(cloud_path, ply, cloud_error))
        {
            LOG_ERROR << "[后台收尾] 点云写入失败: " << cloud_error;
        }
        else if (ctx.manifest_snapshot.is_object())
        {
            // 只改点云那几项，代次与其余内容保持不变：第三流程按代次判重，
            // 天然能接住同一代次的两次发布。
            nlohmann::json document = ctx.manifest_snapshot;
            document["point_cloud"]["status"] = "ready";
            document["point_cloud"]["sha256"] =
                TrajectoryOutputFormatAlgorithm::Sha256Hex(ply);
            // 字段名须与 BuildManifestDocument 的 file_entry 一致（point_count），
            // 写成 item_count 会多出一个野字段、而 point_count 仍停在 0。
            document["point_cloud"]["point_count"] = static_cast<int64_t>(ctx.cloud_b.size());
            std::string publish_error;
            if (!OutputPublisher::PublishManifest(trajectory_output_dir_, document, publish_error))
            {
                LOG_ERROR << "[后台收尾] manifest 补发失败: " << publish_error;
            }
            else
            {
                point_cloud_ready = true;
                LOG_INFO << "[后台收尾] 点云=已写出 点数=" << ctx.cloud_b.size()
                         << " 字节=" << ply.size() << " manifest补发=成功 generation_id="
                         << ctx.generation_id;
            }
        }
    }

    // ---- 后台 2：归档本轮 output（成功或失败都归档；归档失败不改判规划结果）----
    std::vector<std::string> copied_files;
    if (archive.opened && archive_status.status != "failed")
    {
        if (SessionArchive::ArchiveOutputs(archive, output_root_, copied_files, archive_error))
        {
            archive_status.status = "success";
        }
        else
        {
            archive_status.status = "failed";
            archive_status.error = archive_error;
        }
    }

    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - task_started).count();

    // ---- 后台 3：result.json ----
    if (archive.opened)
    {
        nlohmann::json result;
        result["request_id"] = ctx.request_id;
        result["generation_id"] = ctx.generation_id;
        result["workpiece_type"] = ctx.workpiece_type;
        result["planning_status"] = planning_ok ? "success" : "failed";
        result["ready"] = planning_ok && ctx.has_solution;
        result["has_solution"] = ctx.has_solution;
        if (!planning_ok)
        {
            result["failed_flow"] = failed_flow;
            result["failed_stage"] = failed_stage;
            result["reason"] = failure_reason;
        }
        else if (!ctx.has_solution)
        {
            result["reason"] = ctx.failure_reason;
        }
        result["archive_status"] = archive_status.status;
        if (!archive_status.error.empty())
        {
            result["archive_error"] = archive_status.error;
        }
        result["archived_files"] = copied_files;
        // 原始输入按标准图片名保存；request.txt 保留原文，便于回放定位。
        result["input_files"] = {"request.txt", "raw_depth.tiff", "raw_depth_left.tiff",
                                 "raw_depth_right.tiff", "raw_color.png"};
        result["stage_timings_ms"] = ctx.stage_timings_ms;
        // 到客户端可执行的耗时与含后台收尾的总耗时分开记，便于对账。
        result["reply_ms"] = reply_ms;
        result["total_ms"] = total_ms;
        result["point_cloud_ready"] = point_cloud_ready;
        std::string result_error;
        if (!SessionArchive::WriteResult(archive, result, result_error))
        {
            LOG_ERROR << "[本轮完成] result.json 写入失败: " << result_error;
            if (archive_status.status == "success")
            {
                archive_status.status = "failed";
                archive_status.error = "result.json 写入失败: " + result_error;
            }
        }
    }

    LOG_INFO << "[本轮完成] 规划结果=" << (planning_ok ? (ctx.has_solution ? "成功" : "无解") : "异常失败")
             << " 归档结果=" << archive_status.status << " 归档目录=" << archive_status.path
             << " 可执行耗时=" << reply_ms << "ms 含收尾总耗时=" << total_ms << "ms";

    // ---- 第三行：archived ----
    // 客户端可能已在第二行后断开，此时发不出去只记日志，**不改变已发布的轨迹**；
    // 客户端也不能仅凭"没收到 archived"判断归档失败。
    const std::string archived_reply = ResponseBuilder::Archived(
        ctx.request_id, ctx.generation_id, point_cloud_ready, archive_status);
    const bool archived_sent = sender(archived_reply);
    LOG_INFO << "[tcp][收尾应答] archived archive_status=" << archive_status.status
             << " point_cloud_ready=" << (point_cloud_ready ? "true" : "false")
             << " 发送=" << (archived_sent ? "成功" : "失败（客户端已断开，不影响已发布轨迹）");
    LOG_INFO << "[pipeline][完成] 结果=" << (completed ? "成功" : (planning_ok ? "无解" : "失败"))
             << " 总耗时=" << total_ms << "ms";
    // 归档与最终应答都结束后才释放单任务占用（由调用方的锁作用域保证）。
    SessionArchive::EndTrigger(archive);
}

} // namespace openmind::trajectory_plan
