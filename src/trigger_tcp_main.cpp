/**
 * @file trigger_tcp_main.cpp
 * @brief 外部触发客户端：读 input/ 的文字、三张深度图与主 RGB 图，按 GP02 协议发一次任务并等三次应答。
 *
 * 协议（文档「TCP 请求协议」）：固定 24 字节报头 + 五段负载。
 *
 * | 顺序 | 内容                | 长度／编码                |
 * | ---- | ------------------- | ------------------------- |
 * | 1    | 协议标识            | 4 字节 "GP02"             |
 * | 2    | request.txt 字节数  | uint32，网络字节序        |
 * | 3~5  | 主／左／右图字节数  | uint32，网络字节序        |
 * | 6    | 主 RGB 图字节数      | uint32                    |
 * | 7    | request.txt 原始内容 | UTF-8                     |
 * | 8~10 | 主／左／右图 TIFF    | 原始字节，不转 Base64     |
 * | 11   | 主 RGB PNG           | 原始字节，不转 Base64     |
 *
 * 客户端先把文字、三张深度图与主 RGB 图全部读入内存再发送，保证同轮数据一致；
 * 发送后等待 accepted，再等 completed／failed（**到此即可驱动机械臂**），
 * 最后等 archived（点云、归档、result.json 收尾结果）。改动六把应答由两行改为三行。
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <chrono>
#include <array>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "trajectory_plan/algorithm/pose.h"
#include "trajectory_plan/tools/json_util.h"

namespace
{

constexpr char kMagic[4] = {'G', 'P', '0', '2'};

/** @brief 循环发送，直到全部字节送出。 */
bool SendAll(int fd, const uint8_t* data, size_t size)
{
    size_t offset = 0;
    while (offset < size)
    {
        const ssize_t sent = ::send(fd, data + offset, size - offset, MSG_NOSIGNAL);
        if (sent <= 0)
        {
            return false;
        }
        offset += static_cast<size_t>(sent);
    }
    return true;
}

/** @brief 收一行 UTF-8 JSON 应答（以 '\n' 结尾）。 */
bool RecvLine(int fd, std::string& line)
{
    line.clear();
    char byte = 0;
    while (true)
    {
        const ssize_t got = ::recv(fd, &byte, 1, 0);
        if (got <= 0)
        {
            return false;
        }
        if (byte == '\n')
        {
            return true;
        }
        line.push_back(byte);
    }
}

/** @brief 追加 4 字节网络字节序无符号整数。 */
void AppendBe32(std::vector<uint8_t>& buffer, uint32_t value)
{
    buffer.push_back(static_cast<uint8_t>((value >> 24) & 0xff));
    buffer.push_back(static_cast<uint8_t>((value >> 16) & 0xff));
    buffer.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
    buffer.push_back(static_cast<uint8_t>(value & 0xff));
}

/** @brief 原样读入文件的全部字节。 */
bool ReadBytes(const std::string& path, std::string& bytes)
{
    std::string error;
    return openmind::trajectory_plan::ReadTextFile(path, bytes, error);
}

} // namespace

int main(int argc, char* argv[])
{
    std::string input_dir = "input";
    std::string host = "127.0.0.1";
    uint16_t port = 9108;
    int32_t timeout_s = 600;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--input-dir" && index + 1 < argc)
        {
            input_dir = argv[++index];
        }
        else if (argument == "--host" && index + 1 < argc)
        {
            host = argv[++index];
        }
        else if (argument == "--port" && index + 1 < argc)
        {
            port = static_cast<uint16_t>(std::stoi(argv[++index]));
        }
        else if (argument == "--timeout-s" && index + 1 < argc)
        {
            timeout_s = std::stoi(argv[++index]);
        }
        else
        {
            std::cerr << "用法: " << argv[0]
                      << " [--input-dir input] [--host 127.0.0.1] [--port 9108] [--timeout-s 600]"
                      << std::endl;
            return 2;
        }
    }

    // ---- [发送1/3] 读 request.txt ----
    std::string request_text;
    if (!ReadBytes(openmind::trajectory_plan::JoinPath(input_dir, "request.txt"), request_text))
    {
        std::cerr << "[客户端][发送1/3] 读取 request.txt 失败" << std::endl;
        return 1;
    }
    std::string workpiece_type;
    Eigen::Matrix4d t_b_o = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d t_base_flange = Eigen::Matrix4d::Identity();
    std::array<double, 6> t_base_flange_joint_angles_deg {};
    std::string main_rel;
    std::string left_rel;
    std::string right_rel;
    std::string rgb_main_rel;
    std::string error;
    if (!openmind::trajectory_plan::ParseRequestText(request_text, workpiece_type, t_b_o,
                                                     t_base_flange, t_base_flange_joint_angles_deg,
                                                     main_rel, left_rel, right_rel, rgb_main_rel, error))
    {
        std::cerr << "[客户端][发送1/3] request.txt 解析失败: " << error << std::endl;
        return 1;
    }
    std::cout << "[客户端][发送1/3] 读取request.txt完成 工件=" << workpiece_type << std::endl;

    // ---- [发送2/3] 读三张深度图与主 RGB 图（图片路径相对 request.txt 所在目录解析）----
    std::string main_bytes;
    std::string left_bytes;
    std::string right_bytes;
    std::string rgb_main_bytes;
    if (!ReadBytes(openmind::trajectory_plan::JoinPath(input_dir, main_rel), main_bytes) ||
        !ReadBytes(openmind::trajectory_plan::JoinPath(input_dir, left_rel), left_bytes) ||
        !ReadBytes(openmind::trajectory_plan::JoinPath(input_dir, right_rel), right_bytes) ||
        !ReadBytes(openmind::trajectory_plan::JoinPath(input_dir, rgb_main_rel), rgb_main_bytes))
    {
        std::cerr << "[客户端][发送2/3] 读取深度图或主 RGB 图失败" << std::endl;
        return 1;
    }
    std::cout << "[客户端][发送2/3] 三张深度图与主 RGB 图读取完成 主=" << main_bytes.size()
              << "bytes 左=" << left_bytes.size() << "bytes 右=" << right_bytes.size()
              << "bytes RGB=" << rgb_main_bytes.size() << "bytes"
              << std::endl;

    // ---- [发送3/3] 组帧并发送 ----
    std::vector<uint8_t> header;
    header.reserve(24);
    header.insert(header.end(), kMagic, kMagic + 4);
    AppendBe32(header, static_cast<uint32_t>(request_text.size()));
    AppendBe32(header, static_cast<uint32_t>(main_bytes.size()));
    AppendBe32(header, static_cast<uint32_t>(left_bytes.size()));
    AppendBe32(header, static_cast<uint32_t>(right_bytes.size()));
    AppendBe32(header, static_cast<uint32_t>(rgb_main_bytes.size()));

    const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        std::cerr << "创建 socket 失败" << std::endl;
        return 1;
    }
    // 规划可能耗时较久，收应答的超时要留足。
    timeval timeout {};
    timeout.tv_sec = timeout_s;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    int no_delay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1 ||
        ::connect(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    {
        std::cerr << "连接 " << host << ":" << port << " 失败: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }

    const auto started = std::chrono::steady_clock::now();
    const bool sent =
        SendAll(fd, header.data(), header.size()) &&
        SendAll(fd, reinterpret_cast<const uint8_t*>(request_text.data()), request_text.size()) &&
        SendAll(fd, reinterpret_cast<const uint8_t*>(main_bytes.data()), main_bytes.size()) &&
        SendAll(fd, reinterpret_cast<const uint8_t*>(left_bytes.data()), left_bytes.size()) &&
        SendAll(fd, reinterpret_cast<const uint8_t*>(right_bytes.data()), right_bytes.size()) &&
        SendAll(fd, reinterpret_cast<const uint8_t*>(rgb_main_bytes.data()), rgb_main_bytes.size());
    if (!sent)
    {
        std::cerr << "[客户端][发送3/3] 发送失败: " << std::strerror(errno) << std::endl;
        ::close(fd);
        return 1;
    }
    const size_t total = header.size() + request_text.size() + main_bytes.size() + left_bytes.size() +
                         right_bytes.size() + rgb_main_bytes.size();
    const double send_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    std::cout << "[客户端][发送3/3] TCP发送完成 总字节数=" << total << " 耗时=" << send_ms << "ms"
              << std::endl;

    // ---- 等第一行应答：accepted / rejected / busy ----
    std::string first_line;
    if (!RecvLine(fd, first_line))
    {
        std::cerr << "[客户端] 未收到首次应答（超时或断连）" << std::endl;
        ::close(fd);
        return 1;
    }
    std::cout << "[客户端][应答1] " << first_line << std::endl;
    if (first_line.find("\"status\":\"accepted\"") == std::string::npos)
    {
        // rejected / busy：任务未被接受，不会有最终应答。
        std::cout << "[客户端][完成] 结果=未被接受" << std::endl;
        ::close(fd);
        return 3;
    }

    // ---- 等第二行应答：completed / failed。到此客户端即可驱动机械臂 ----
    std::string final_line;
    if (!RecvLine(fd, final_line))
    {
        // 客户端断线后不能仅凭"没有收到应答"判断任务没有执行。
        std::cerr << "[客户端] 未收到轨迹应答；任务已被接受，服务端仍会继续处理并落盘"
                  << std::endl;
        ::close(fd);
        return 1;
    }
    const double reply_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    std::cout << "[客户端][应答2] " << final_line << std::endl;
    const bool completed = final_line.find("\"status\":\"completed\"") != std::string::npos;
    const bool ready = final_line.find("\"ready\":true") != std::string::npos;
    std::cout << "[客户端][轨迹就绪] 结果=" << (completed ? (ready ? "成功" : "无解") : "失败")
              << " 输出=service侧output/ 到可执行耗时=" << reply_ms << "ms" << std::endl;

    // ---- 等第三行应答：archived。点云、归档、result.json 收尾 ----
    // 真实客户端到第二行就可以断开去驱动机械臂；这里等第三行是为了完整验证协议。
    // 注意：**收不到 archived 不等于归档失败**，只说明本连接没等到收尾。
    std::string archived_line;
    if (!RecvLine(fd, archived_line))
    {
        std::cerr << "[客户端] 未收到收尾应答；轨迹已发布且可执行，"
                     "服务端仍会继续写点云与归档" << std::endl;
        ::close(fd);
        return completed ? 0 : 4;
    }
    const double total_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    std::cout << "[客户端][应答3] " << archived_line << std::endl;
    std::cout << "[客户端][完成] 到可执行=" << reply_ms << "ms 含后台收尾总耗时="
              << total_ms << "ms" << std::endl;
    ::close(fd);
    return completed ? 0 : 4;
}
