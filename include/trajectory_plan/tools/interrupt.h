#ifndef HYBRID_TRAJECTORY_PLAN_INTERRUPT_H
#define HYBRID_TRAJECTORY_PLAN_INTERRUPT_H

/**
 * @file interrupt.h
 * @brief 信号处理：SIGINT / SIGTERM 置原子停止标志，主循环据此优雅退出。
 */

namespace openmind::trajectory_plan
{

/**
 * @brief 注册信号处理并安装停止标志；重复调用安全。
 */
void InstallSignalHandlers();

/**
 * @brief 是否收到停止信号。
 */
bool IsInterrupted();

/**
 * @brief 重置停止标志（测试用）。
 */
void ResetInterruptFlag();

/**
 * @brief 阻塞等待停止信号。
 */
void WaitForInterrupt();

} // namespace openmind::trajectory_plan

#endif // HYBRID_TRAJECTORY_PLAN_INTERRUPT_H
