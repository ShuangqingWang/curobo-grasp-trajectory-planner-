/**
 * @file interrupt.cpp
 * @brief 信号处理实现：SIGINT/SIGTERM → 原子停止标志。
 */

#include "trajectory_plan/tools/interrupt.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

namespace openmind::trajectory_plan
{
namespace
{

std::atomic<bool> g_interrupted{false};

void HandleSignal(int signal_number)
{
    // 信号处理器内只做最小动作：置原子标志，主循环轮询退出。
    (void)signal_number;
    g_interrupted.store(true, std::memory_order_relaxed);
}

} // namespace

void InstallSignalHandlers()
{
    // 重复注册安全；两个信号共用同一处理器。
    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
}

bool IsInterrupted()
{
    return g_interrupted.load(std::memory_order_relaxed);
}

void ResetInterruptFlag()
{
    g_interrupted.store(false, std::memory_order_relaxed);
}

void WaitForInterrupt()
{
    while (!IsInterrupted())
    {
        // 100 ms 轮询粒度：信号不打断本线程，轮询保证及时退出且不占 CPU。
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

} // namespace openmind::trajectory_plan
