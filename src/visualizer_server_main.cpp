/**
 * @file visualizer_server_main.cpp
 * @brief 仿真静态文件服务 + 只读绝对路径读取。
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#include <climits>
#include <cstdlib>

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace
{

bool EndsWith(const std::string& text, const std::string& suffix)
{
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string UrlDecode(const std::string& text)
{
    std::string out;
    for (size_t index = 0; index < text.size(); ++index)
    {
        if (text[index] == '%' && index + 2 < text.size())
        {
            out.push_back(static_cast<char>(std::stoi(text.substr(index + 1, 2), nullptr, 16)));
            index += 2;
        }
        else if (text[index] == '+')
        {
            out.push_back(' ');
        }
        else
        {
            out.push_back(text[index]);
        }
    }
    return out;
}

std::string QueryValue(const std::string& query, const std::string& key)
{
    std::string token = key + "=";
    size_t start = 0;
    while (start < query.size())
    {
        const size_t amp = query.find('&', start);
        const std::string part = query.substr(start, amp == std::string::npos ? std::string::npos : amp - start);
        if (part.rfind(token, 0) == 0)
        {
            return UrlDecode(part.substr(token.size()));
        }
        if (amp == std::string::npos)
        {
            break;
        }
        start = amp + 1;
    }
    return "";
}

std::string MimeType(const std::string& path)
{
    if (EndsWith(path, ".html"))
    {
        return "text/html; charset=utf-8";
    }
    if (EndsWith(path, ".js"))
    {
        return "application/javascript; charset=utf-8";
    }
    if (EndsWith(path, ".css"))
    {
        return "text/css; charset=utf-8";
    }
    if (EndsWith(path, ".json"))
    {
        return "application/json; charset=utf-8";
    }
    if (EndsWith(path, ".ply"))
    {
        return "application/octet-stream";
    }
    if (EndsWith(path, ".yaml") || EndsWith(path, ".yml"))
    {
        return "text/yaml; charset=utf-8";
    }
    return "application/octet-stream";
}

bool AllowedAbsolute(const std::string& path)
{
    return EndsWith(path, ".csv") || EndsWith(path, ".json") || EndsWith(path, ".ply") ||
           EndsWith(path, ".txt") || EndsWith(path, ".yaml") || EndsWith(path, ".yml");
}

bool ReadFile(const std::string& path, std::string& body)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
    {
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    body = buffer.str();
    return true;
}

void Send(int fd, const std::string& status, const std::string& type, const std::string& body)
{
    std::ostringstream response;
    response << "HTTP/1.1 " << status << "\r\n"
             << "Content-Type: " << type << "\r\n"
             << "Content-Length: " << body.size() << "\r\n"
             << "Cache-Control: no-store\r\n"
             << "Connection: close\r\n\r\n"
             << body;
    const std::string text = response.str();
    ::send(fd, text.data(), text.size(), 0);
}

void Handle(int fd, const std::string& root)
{
    char buffer[8192];
    const ssize_t got = ::recv(fd, buffer, sizeof(buffer) - 1, 0);
    if (got <= 0)
    {
        return;
    }
    buffer[got] = '\0';
    std::istringstream request(buffer);
    std::string method;
    std::string target;
    std::string version;
    request >> method >> target >> version;
    const size_t qpos = target.find('?');
    const std::string path = qpos == std::string::npos ? target : target.substr(0, qpos);
    const std::string query = qpos == std::string::npos ? "" : target.substr(qpos + 1);

    if (path == "/__visualizer_project_root__")
    {
        // 第三流程的「项目相对路径」以可视化服务确定的本项目根目录为基准，
        // 不以网页所在的 scripts/simulation/ 或浏览器电脑目录为基准。
        std::string absolute_root = root;
        char resolved[PATH_MAX] {};
        if (realpath(root.c_str(), resolved) != nullptr)
        {
            absolute_root = resolved;
        }
        Send(fd, "200 OK", "application/json",
             std::string("{\"success\":true,\"project_root\":\"") + absolute_root + "\"}\n");
        return;
    }

    if (path == "/__visualizer_absolute_file__")
    {
        const std::string raw = QueryValue(query, "path");
        if (raw.empty() || raw.front() != '/')
        {
            Send(fd, "400 Bad Request", "application/json",
                 "{\"success\":false,\"error\":\"必须填写绝对路径\"}\n");
            return;
        }
        if (!AllowedAbsolute(raw))
        {
            Send(fd, "403 Forbidden", "application/json",
                 "{\"success\":false,\"error\":\"不允许读取该文件类型\"}\n");
            return;
        }
        std::string body;
        if (!ReadFile(raw, body))
        {
            Send(fd, "404 Not Found", "application/json",
                 "{\"success\":false,\"error\":\"文件不存在或不可读\"}\n");
            return;
        }
        Send(fd, "200 OK", MimeType(raw), body);
        return;
    }

    std::string relative = path;
    if (relative.empty() || relative == "/")
    {
        relative = "/index.html";
    }
    if (relative.find("..") != std::string::npos)
    {
        Send(fd, "403 Forbidden", "text/plain", "forbidden");
        return;
    }
    std::string file_path = root + relative;
    if (EndsWith(file_path, "/"))
    {
        file_path += "index.html";
    }
    std::string body;
    if (!ReadFile(file_path, body))
    {
        Send(fd, "404 Not Found", "text/plain", "not found");
        return;
    }
    Send(fd, "200 OK", MimeType(file_path), body);
}

} // namespace

int main(int argc, char* argv[])
{
    std::string host = "127.0.0.1";
    int port = 8088;
    std::string directory = ".";
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--host" && index + 1 < argc)
        {
            host = argv[++index];
        }
        else if (argument == "--port" && index + 1 < argc)
        {
            port = std::stoi(argv[++index]);
        }
        else if (argument == "--directory" && index + 1 < argc)
        {
            directory = argv[++index];
        }
    }

    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0)
    {
        std::cerr << "socket 失败\n";
        return 1;
    }
    int reuse = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    sockaddr_in address {};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1)
    {
        std::cerr << "非法 host\n";
        return 1;
    }
    int bound_port = port;
    while (bound_port < 9000)
    {
        address.sin_port = htons(static_cast<uint16_t>(bound_port));
        if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0)
        {
            break;
        }
        ++bound_port;
    }
    if (bound_port >= 9000)
    {
        std::cerr << "8088..8999 没有空闲端口\n";
        return 1;
    }
    if (::listen(listen_fd, 16) != 0)
    {
        std::cerr << "listen 失败\n";
        return 1;
    }
    std::cout << "visualizer listening http://" << host << ":" << bound_port << std::endl;
    while (true)
    {
        const int client = ::accept(listen_fd, nullptr, nullptr);
        if (client < 0)
        {
            continue;
        }
        Handle(client, directory);
        ::close(client);
    }
}
