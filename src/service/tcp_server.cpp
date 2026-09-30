/**
 * @file tcp_server.cpp
 * @brief 外部触发 TCP 服务实现：24 字节报头 + 五段负载，按长度循环接收。
 */

#include "trajectory_plan/service/tcp_server.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <sstream>

#include "trajectory_plan/tools/log.h"

namespace openmind::trajectory_plan
{
namespace
{

constexpr size_t kMagicBytes = 4;
constexpr size_t kHeaderBytes = 24; ///< 4 标识 + 5×uint32

/**
 * @brief 从缓冲区读 4 字节网络字节序无符号整数。
 */
uint32_t ReadBigEndianUint32(const uint8_t* bytes)
{
    return (static_cast<uint32_t>(bytes[0]) << 24) | (static_cast<uint32_t>(bytes[1]) << 16) |
           (static_cast<uint32_t>(bytes[2]) << 8) | static_cast<uint32_t>(bytes[3]);
}

/**
 * @brief 设置收发超时，避免半截报文把连接线程永久挂住。
 */
void ApplySocketTimeout(int socket_fd, int32_t timeout_ms)
{
    if (timeout_ms <= 0)
    {
        return;
    }
    timeval timeout {};
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
}

/**
 * @brief 报文层拒绝：此时还没有 request_id，只回 rejected 与原因。
 */
void SendProtocolRejection(const ResponseSender& sender, const std::string& reason)
{
    std::string escaped;
    escaped.reserve(reason.size() + 8);
    for (const char character : reason)
    {
        if (character == '"' || character == '\\')
        {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    sender("{\"status\":\"rejected\",\"request_id\":\"\",\"reason\":\"" + escaped + "\"}");
}

} // namespace

TcpServer::~TcpServer()
{
    Stop();
}

bool TcpServer::Start(const std::string& host,
                      uint16_t port,
                      const TcpServerLimits& limits,
                      FrameHandler handler,
                      std::string& error)
{
    if (running_)
    {
        error = "TCP 服务已在运行";
        return false;
    }
    if (limits.protocol_magic.size() != kMagicBytes)
    {
        error = "协议标识必须是 4 字节: " + limits.protocol_magic;
        return false;
    }
    listen_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0)
    {
        error = std::string("创建 socket 失败: ") + std::strerror(errno);
        return false;
    }
    int reuse = 1;
    setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(port);
    if (inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
    {
        error = "绑定地址无效: " + host;
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    // 禁止同端口起两份：绑定失败即启动失败，明确报错退出。
    if (bind(listen_fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
    {
        error = std::string("绑定端口失败（可能已有同端口进程）: ") + std::strerror(errno);
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (listen(listen_fd_, 8) != 0)
    {
        error = std::string("监听失败: ") + std::strerror(errno);
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }

    limits_ = limits;
    handler_ = std::move(handler);
    port_ = port;
    running_ = true;
    accept_thread_ = std::thread(&TcpServer::AcceptLoop, this);
    return true;
}

void TcpServer::Stop()
{
    if (!running_)
    {
        return;
    }
    running_ = false;
    // shutdown 唤醒 accept 阻塞，使线程尽快退出。
    if (listen_fd_ >= 0)
    {
        ::shutdown(listen_fd_, SHUT_RDWR);
    }
    if (accept_thread_.joinable())
    {
        accept_thread_.join();
    }
    // 已 accepted 的任务必须跑完并落盘，等待在处理的连接线程收尾。
    while (active_connections_.load() > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (listen_fd_ >= 0)
    {
        ::close(listen_fd_);
        listen_fd_ = -1;
    }
}

bool TcpServer::IsRunning() const
{
    return running_;
}

uint16_t TcpServer::Port() const
{
    return port_;
}

void TcpServer::AcceptLoop()
{
    while (running_)
    {
        sockaddr_in client_address {};
        socklen_t address_length = sizeof(client_address);
        const int client_fd = accept(listen_fd_, reinterpret_cast<sockaddr*>(&client_address), &address_length);
        if (client_fd < 0)
        {
            if (running_)
            {
                LOG_WARN << "[tcp] accept 失败: " << std::strerror(errno);
            }
            continue;
        }
        char peer_text[INET_ADDRSTRLEN] {};
        inet_ntop(AF_INET, &client_address.sin_addr, peer_text, sizeof(peer_text));
        std::ostringstream peer;
        peer << peer_text << ":" << ntohs(client_address.sin_port);
        const uint64_t sequence = ++connection_sequence_;

        // 每个连接独立线程：任务执行期间新连接仍能立刻拿到 busy，不排队。
        ++active_connections_;
        std::thread([this, client_fd, peer_string = peer.str(), sequence]() {
            HandleConnection(client_fd, peer_string, sequence);
            ::close(client_fd);
            --active_connections_;
        }).detach();
    }
}

bool TcpServer::ReceiveFrame(int socket_fd, TriggerFrame& frame, std::string& error) const
{
    uint8_t header[kHeaderBytes] {};
    if (!ReadExact(socket_fd, header, sizeof(header)))
    {
        error = "报头接收不完整或超时";
        return false;
    }
    if (std::memcmp(header, limits_.protocol_magic.data(), kMagicBytes) != 0)
    {
        error = "协议标识非法，期望 " + limits_.protocol_magic;
        return false;
    }
    const uint64_t text_bytes = ReadBigEndianUint32(header + 4);
    const uint64_t main_bytes = ReadBigEndianUint32(header + 8);
    const uint64_t left_bytes = ReadBigEndianUint32(header + 12);
    const uint64_t right_bytes = ReadBigEndianUint32(header + 16);
    const uint64_t rgb_main_bytes = ReadBigEndianUint32(header + 20);

    // 先按报头声明的长度逐项校验，绝不因为报头声明就无条件分配内存。
    if (text_bytes == 0 || text_bytes > limits_.max_request_text_bytes)
    {
        error = "request.txt 字节数非法: " + std::to_string(text_bytes);
        return false;
    }
    const uint64_t image_sizes[3] = {main_bytes, left_bytes, right_bytes};
    const char* image_names[3] = {"主图", "左图", "右图"};
    for (int32_t index = 0; index < 3; ++index)
    {
        if (image_sizes[index] == 0 || image_sizes[index] > limits_.max_depth_image_bytes)
        {
            error = std::string(image_names[index]) + "字节数非法: " + std::to_string(image_sizes[index]);
            return false;
        }
    }
    if (rgb_main_bytes == 0 || rgb_main_bytes > limits_.max_depth_image_bytes)
    {
        error = "主 RGB 图字节数非法: " + std::to_string(rgb_main_bytes);
        return false;
    }
    const uint64_t total = text_bytes + main_bytes + left_bytes + right_bytes + rgb_main_bytes;
    if (total > limits_.max_total_frame_bytes)
    {
        error = "报文总长度超过上限: " + std::to_string(total);
        return false;
    }
    LOG_INFO << "[tcp][接收1/3] 报头校验通过 连接=" << frame.connection_id
             << " 协议=" << limits_.protocol_magic << " 文字=" << text_bytes
             << "B 主=" << main_bytes << "B 左=" << left_bytes << "B 右=" << right_bytes
             << "B RGB=" << rgb_main_bytes << "B";

    const auto started = std::chrono::steady_clock::now();
    frame.request_text.resize(static_cast<size_t>(text_bytes));
    if (!ReadExact(socket_fd, frame.request_text.data(), frame.request_text.size()))
    {
        error = "request.txt 接收不完整或超时";
        return false;
    }
    std::vector<uint8_t>* buffers[3] = {&frame.depth_main, &frame.depth_left, &frame.depth_right};
    for (int32_t index = 0; index < 3; ++index)
    {
        buffers[index]->resize(static_cast<size_t>(image_sizes[index]));
        if (!ReadExact(socket_fd, buffers[index]->data(), buffers[index]->size()))
        {
            error = std::string(image_names[index]) + "接收不完整或超时";
            return false;
        }
    }
    frame.rgb_main.resize(static_cast<size_t>(rgb_main_bytes));
    if (!ReadExact(socket_fd, frame.rgb_main.data(), frame.rgb_main.size()))
    {
        error = "主 RGB 图接收不完整或超时";
        return false;
    }
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count();
    LOG_INFO << "[tcp][接收2/3] 文字、三张深度图与主 RGB 图接收完整 连接=" << frame.connection_id
             << " 总字节数=" << total << " 耗时=" << elapsed_ms << "ms";
    return true;
}

void TcpServer::HandleConnection(int socket_fd, const std::string& peer, uint64_t connection_sequence)
{
    ApplySocketTimeout(socket_fd, limits_.receive_timeout_ms);
    int no_delay = 1;
    setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));

    TriggerFrame frame;
    frame.connection_id = "conn_" + std::to_string(connection_sequence);
    frame.peer = peer;

    // 应答发送器：一行 JSON + '\n'；发送失败只记录，不改变已发布的结果。
    const ResponseSender sender = [socket_fd, &frame](const std::string& line) {
        const std::string payload = line + "\n";
        size_t sent = 0;
        while (sent < payload.size())
        {
            const ssize_t written =
                ::send(socket_fd, payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
            if (written <= 0)
            {
                LOG_WARN << "[tcp][应答] 发送失败 连接=" << frame.connection_id
                         << " 内容=" << line;
                return false;
            }
            sent += static_cast<size_t>(written);
        }
        return true;
    };

    std::string error;
    if (!ReceiveFrame(socket_fd, frame, error))
    {
        LOG_WARN << "[tcp][接收失败] 连接=" << frame.connection_id << " 对端=" << peer
                 << " 原因=" << error;
        SendProtocolRejection(sender, error);
        // 拒绝往往发生在客户端还在发后续负载的时候。直接关闭会让内核回 RST，
        // 把已经写出的 rejected 应答一起丢掉；先半关写端再把剩余入站数据读干净，
        // 客户端才能真正收到拒绝原因。
        DrainAndShutdown(socket_fd);
        return;
    }
    if (handler_)
    {
        handler_(frame, sender);
    }
    DrainAndShutdown(socket_fd);
}

void TcpServer::DrainAndShutdown(int socket_fd)
{
    ::shutdown(socket_fd, SHUT_WR);
    uint8_t sink[64 * 1024];
    // 有界读取：对端正常结束会很快 EOF；异常情况下也不会在这里无限等待。
    for (int32_t round = 0; round < 4096; ++round)
    {
        const ssize_t received = ::recv(socket_fd, sink, sizeof(sink), 0);
        if (received <= 0)
        {
            return;
        }
    }
}

bool TcpServer::ReadExact(int socket_fd, void* buffer, size_t count)
{
    auto* cursor = static_cast<uint8_t*>(buffer);
    size_t remaining = count;
    while (remaining > 0)
    {
        const ssize_t received = recv(socket_fd, cursor, remaining, 0);
        if (received <= 0)
        {
            return false;
        }
        cursor += received;
        remaining -= static_cast<size_t>(received);
    }
    return true;
}

} // namespace openmind::trajectory_plan
