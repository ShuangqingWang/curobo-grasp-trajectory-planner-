#ifndef HYBRID_TRAJECTORY_PLAN_TCP_SERVER_H
#define HYBRID_TRAJECTORY_PLAN_TCP_SERVER_H

/**
 * @file tcp_server.h
 * @brief 外部触发 TCP 服务：固定 24 字节报头 + request.txt 原文 + 三张 TIFF 与主 RGB 原始字节。
 *
 * 协议（文档「TCP 请求协议」，全部整数为网络字节序 uint32）：
 *
 * | 顺序 | 内容                | 长度／编码                       |
 * | ---- | ------------------- | -------------------------------- |
 * | 1    | 协议标识            | 4 字节，当前为 "GP02"            |
 * | 2    | request.txt 字节数  | uint32                           |
 * | 3    | 主图字节数          | uint32                           |
 * | 4    | 左图字节数          | uint32                           |
 * | 5    | 右图字节数          | uint32                           |
 * | 6    | 主 RGB 图字节数      | uint32                           |
 * | 7    | request.txt 原始内容 | UTF-8，长度由报头指定            |
 * | 8~10 | 主／左／右图 TIFF    | 原始字节，长度由报头指定         |
 * | 11   | 主 RGB 图            | PNG 原始字节，长度由报头指定     |
 *
 * 固定报头共 24 字节。TCP 是字节流，按长度循环接收，不把一次 recv 当完整报文。
 * 接收前检查每段长度与总长度上限，并设置接收超时；报文不完整、超时、非法标识或
 * 字段校验失败时拒绝请求，不执行 flow、不替换最近已接受任务的结果。
 *
 * 应答为 UTF-8、每行一个 JSON：先 accepted／rejected／busy，任务完成后再
 * completed／failed。二者通过 ResponseSender 分两次发出。
 */

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace openmind::trajectory_plan
{

/**
 * @brief 接收与执行限制（来自 app.json 的 trigger_tcp 节）。
 */
struct TcpServerLimits
{
    std::string protocol_magic = "GP02";        ///< 4 字节协议标识
    uint64_t max_request_text_bytes = 1048576;  ///< request.txt 上限
    uint64_t max_depth_image_bytes = 134217728; ///< 单张深度图上限
    uint64_t max_total_frame_bytes = 536870912; ///< 五段负载总字节上限
    int32_t receive_timeout_ms = 30000;         ///< 单次 recv 超时
};

/**
 * @brief 一次触发收到的完整内容：request.txt 原文 + 三张 TIFF 与主 RGB 原始字节。
 *
 * 计算使用这里的内存数据；文字中的图片路径仅供客户端定位文件，
 * service 不按这些路径读本地图片。
 */
struct TriggerFrame
{
    std::string connection_id;          ///< 连接标识（建任务前的日志关联）
    std::string peer;                   ///< 对端地址，日志用
    std::string request_text;           ///< request.txt 原始 UTF-8 文本
    std::vector<uint8_t> depth_main;    ///< 主图 TIFF 原始字节
    std::vector<uint8_t> depth_left;    ///< 左图 TIFF 原始字节
    std::vector<uint8_t> depth_right;   ///< 右图 TIFF 原始字节
    std::vector<uint8_t> rgb_main;      ///< 主图 RGB PNG 原始字节
};

/**
 * @brief 应答发送器：发送一行 JSON（自动补换行）；发送失败返回 false。
 */
using ResponseSender = std::function<bool(const std::string& json_line)>;

/**
 * @brief 帧处理器：收完一帧后调用，自行决定发几次应答。
 *
 * 约定：先发 accepted／rejected／busy，任务完成后再发 completed／failed。
 * 任务一旦 accepted，客户端断开也继续处理并落盘。
 */
using FrameHandler = std::function<void(const TriggerFrame&, const ResponseSender&)>;

/**
 * @brief 外部触发 TCP 服务。
 *
 * 每个连接在独立线程处理，使任务执行期间新连接仍能立刻收到 busy
 * （是否 busy 由帧处理器判定，服务本身不排队、不缓存请求）。
 */
class TcpServer
{
  public:
    TcpServer() = default;
    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;
    ~TcpServer();

    /**
     * @brief 启动监听。
     * @param host 绑定地址（如 0.0.0.0）
     * @param port 端口（app.json trigger_tcp.port，默认 9108）
     * @param limits 协议标识、长度上限与接收超时
     * @param handler 帧处理器
     * @param error 失败原因（成功时为空；端口被占也在此报错，禁止同端口起两份）
     * @return 成功返回 true
     */
    bool Start(const std::string& host,
               uint16_t port,
               const TcpServerLimits& limits,
               FrameHandler handler,
               std::string& error);

    /** @brief 停止监听并等待线程退出。 */
    void Stop();

    /** @brief 是否在运行。 */
    bool IsRunning() const;

    /** @brief 监听端口。 */
    uint16_t Port() const;

    /**
     * @brief 从 socket 读满 n 字节；失败（断连／超时）返回 false。
     */
    static bool ReadExact(int socket_fd, void* buffer, size_t count);

  private:
    /** @brief accept 循环（后台线程）。 */
    void AcceptLoop();

    /** @brief 处理单个连接：收完整帧 → handler（handler 自行应答）。 */
    void HandleConnection(int socket_fd, const std::string& peer, uint64_t connection_sequence);

    /**
     * @brief 按协议接收一帧；任何长度／标识／完整性问题返回 false 并填 error。
     */
    bool ReceiveFrame(int socket_fd, TriggerFrame& frame, std::string& error) const;

    /**
     * @brief 半关写端并把剩余入站数据读干净，避免内核回 RST 丢掉已发出的应答。
     *
     * 拒绝一帧时客户端往往还在发后续负载；直接 close 会让对端收到 RST 而不是
     * rejected 应答。
     */
    static void DrainAndShutdown(int socket_fd);

    int listen_fd_ = -1;              ///< 监听 socket
    uint16_t port_ = 0;               ///< 监听端口
    std::atomic<bool> running_ {false}; ///< 运行标志
    TcpServerLimits limits_;          ///< 协议限制
    FrameHandler handler_;            ///< 帧处理器
    std::thread accept_thread_;       ///< accept 线程
    std::atomic<uint64_t> connection_sequence_ {0}; ///< 连接序号
    std::atomic<int64_t> active_connections_ {0};   ///< 在处理的连接数（Stop 时等待）
};

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_TCP_SERVER_H
