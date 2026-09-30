/**
 * @file main.cpp
 * @brief 进程入口：argv、chdir、信号、进程名、glog，创建 App 并 Init / Run / Stop。
 */

#include <libgen.h>
#include <sys/prctl.h>
#include <unistd.h>

#include <climits>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "trajectory_plan/openmind_trajectory_plan_app.h"
#include "trajectory_plan/tools/interrupt.h"
#include "trajectory_plan/tools/log.h"

namespace
{

constexpr char kDefaultConfigPath[] = "config/grasp_planner.json";

/**
 * @brief 解析 argv：argv[1] 为启动配置；--dump-config 打印合并配置后退出。
 */
bool ParseArguments(int argc, char* argv[], std::string& config_path, bool& dump_config, bool& once,
                   std::string& input_dir, std::string& log_root)
{
    config_path = kDefaultConfigPath;
    dump_config = false;
    once = false;
    input_dir = "input";
    log_root.clear();
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--dump-config")
        {
            dump_config = true;
        }
        else if (argument == "--once")
        {
            once = true;
        }
        else if (argument == "--input-dir" && index + 1 < argc)
        {
            input_dir = argv[++index];
        }
        else if (argument == "--log-root" && index + 1 < argc)
        {
            // 覆盖 app.json 的 log_root。用于 /var 不可写的开发机；
            // 生产仍按配置使用 /var/hybird_log/grasp_log。
            log_root = argv[++index];
        }
        else if (!argument.empty() && argument.front() == '-')
        {
            std::cerr << "未知参数: " << argument << std::endl;
            std::cerr << "用法: " << argv[0]
                      << " [config/grasp_planner.json] [--dump-config] [--once] [--input-dir input]"
                      << " [--log-root /var/hybird_log/grasp_log]" << std::endl;
            return false;
        }
        else
        {
            config_path = argument;
        }
    }
    return true;
}

/**
 * @brief chdir 到可执行文件目录：配置/输入输出用相对路径，工作目录必须固定。
 */
bool ChangeToProjectRoot(std::string& error)
{
    char executable_path[PATH_MAX] {};
    const ssize_t length = readlink("/proc/self/exe", executable_path, sizeof(executable_path) - 1);
    if (length <= 0)
    {
        error = "无法读取可执行文件路径";
        return false;
    }
    executable_path[length] = '\0';
    char* directory = dirname(executable_path);
    std::string project_root = directory;
    const std::string leaf = project_root.substr(project_root.find_last_of('/') + 1);
    if (leaf == "build")
    {
        const size_t separator = project_root.find_last_of('/');
        if (separator != std::string::npos)
        {
            project_root = project_root.substr(0, separator);
        }
    }
    if (chdir(project_root.c_str()) != 0)
    {
        error = std::string("chdir 失败: ") + std::strerror(errno);
        return false;
    }
    return true;
}

/**
 * @brief 设置进程名（comm 短名）：便于 pgrep -af OpenmindTrajectoryPlan。
 */
void ApplyProcessName(const std::string& name)
{
    prctl(PR_SET_NAME, name.c_str(), 0, 0, 0);
}

} // namespace

int main(int argc, char* argv[])
{
    std::string config_path;
    bool dump_config = false;
    bool once = false;
    std::string input_dir;
    std::string log_root;
    if (!ParseArguments(argc, argv, config_path, dump_config, once, input_dir, log_root))
    {
        return 1;
    }

    char original_cwd[PATH_MAX] {};
    if (getcwd(original_cwd, sizeof(original_cwd)) == nullptr)
    {
        std::cerr << "无法读取工作目录" << std::endl;
        return 1;
    }
    if (!config_path.empty() && config_path.front() != '/')
    {
        config_path = std::string(original_cwd) + "/" + config_path;
    }
    if (!input_dir.empty() && input_dir.front() != '/')
    {
        input_dir = std::string(original_cwd) + "/" + input_dir;
    }
    if (!log_root.empty() && log_root.front() != '/')
    {
        log_root = std::string(original_cwd) + "/" + log_root;
    }

    std::string error;
    if (!ChangeToProjectRoot(error))
    {
        std::cerr << error << std::endl;
        return 1;
    }

    ApplyProcessName("OpenmindTrajectoryPlan");
    openmind::trajectory_plan::InstallSignalHandlers();

    openmind::trajectory_plan::OpenmindTrajectoryPlanApp app;
    if (!app.LoadConfig(config_path, error))
    {
        std::cerr << "加载配置失败: " << error << std::endl;
        return 1;
    }
    if (dump_config)
    {
        app.DumpConfig();
        return 0;
    }
    if (!log_root.empty())
    {
        app.OverrideLogRoot(log_root);
    }
    if (!app.Init(error))
    {
        std::cerr << "初始化失败: " << error << std::endl;
        return 1;
    }
    if (once)
    {
        if (!app.RunOnce(input_dir, error))
        {
            std::cerr << "单次运行失败: " << error << std::endl;
            app.Stop();
            return 1;
        }
        app.Stop();
        return 0;
    }
    if (!app.Run(error))
    {
        std::cerr << "运行失败: " << error << std::endl;
        app.Stop();
        return 1;
    }
    app.Stop();
    return 0;
}
