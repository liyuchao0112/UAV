/**
 * @file pyro_autoaim_com.cpp
 * @brief Autoaim Application Task Framework
 * * 负责处理与 MiniPC 的上层业务逻辑：
 * 1. 定期将云台姿态打包发送给 PC
 * 2. 接收 PC 传回的自瞄目标角度并下发给云台控制器
 */

#include "pyro_autoaim_drv.h"
#include "pyro_module_base.h"
// #include "pyro_screw_gimbal.h" // 引入你的云台头文件获取/设置姿态
// 引入其他需要的头文件...

using namespace pyro;

/* ========================================================================== */
/* Static Variables                                                           */
/* ========================================================================== */
static TaskHandle_t autoaim_app_task_handle = nullptr;
static pyro::autoaim_drv_t *autoaim_drv_ptr = nullptr;

/* ========================================================================== */
/* Helper Function Declarations                                               */
/* ========================================================================== */
static void process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t& rx_data);
static void update_and_send_feedback();

/* ========================================================================== */
/* FreeRTOS Task Entries                                                      */
/* ========================================================================== */
extern "C"
{
    /**
     * @brief 自动瞄准应用层主循环线程
     */
    void hero_autoaim_app_thread(void *argument)
    {
        // 延时等待底层设备初始化完成
        vTaskDelay(pdMS_TO_TICKS(500));

        while (true)
        {
            // 1. 检查 PC 视觉是否在线
            if (autoaim_drv_ptr->check_online())
            {
                // 获取 PC 下发的目标数据
                const auto& rx_data = autoaim_drv_ptr->get_target_data();

                // 处理接收到的数据 (例如：将目标角度传递给云台 command)
                process_pc_target_data(rx_data);
            }
            else
            {
                // PC 离线时的安全处理逻辑 (例如：清除自瞄标志位，切换回纯手控)
            }

            // 2. 无论是否收到数据，都可以按照一定频率(如1kHz)向PC发送当前云台姿态
            update_and_send_feedback();

            // 3. 延时让出 CPU
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }

    /**
     * @brief 自动瞄准应用层初始化函数 (由 FreeRTOS 启动时调用)
     */
    void hero_autoaim_app_init(void *argument)
    {
        // 1. 获取底层驱动实例
        autoaim_drv_ptr = &pyro::autoaim_drv_t::get_instance();

        // 2. 启动驱动层的接收和解析任务
        autoaim_drv_ptr->start_rx();

        // 3. 创建应用层业务线程
        // 优先级可以设置得相对较高，但一般低于云台电机控制线程
        xTaskCreate(hero_autoaim_app_thread, "autoaim_app_thread", 256, nullptr,
                    configMAX_PRIORITIES - 3, &autoaim_app_task_handle);

        // 4. 初始化完成，删除自身
        vTaskDelete(nullptr);
    }
}

/* ========================================================================== */
/* Helper Function Implementations                                            */
/* ========================================================================== */

/**
 * @brief 解析并处理来自 PC 的自瞄控制数据
 */
static void process_pc_target_data(const pyro::autoaim_drv_t::rx_data_t& rx_data)
{
    // TODO: 解析 rx_data
    // 示例:
    // if (rx_data.fire) { ... }
    // float target_yaw = rx_data.shoot_yaw;
    // float target_pitch = rx_data.shoot_pitch;

    // TODO: 将角度传给云台 cmd，或者更新底盘的移动预测等
}

/**
 * @brief 获取当前 MCU 状态，并将其发送给 PC
 */
static void update_and_send_feedback()
{
    // 1. 获取待发送数据的引用 (这是咱们刚刚重构的新接口)
    auto& tx_data = autoaim_drv_ptr->get_tx_data();

    // 2. 填充数据
    // TODO: 从云台控制上下文中获取真实的 yaw, pitch, 速度, 颜色等
    // tx_data.curr_yaw = ...;
    // tx_data.curr_pitch = ...;
    // tx_data.state = ...;
    // tx_data.autoaim = ...;
    // tx_data.enemy_color = ...;

    // 3. 调用无参发送接口，底层自动计算 CRC 和封包发送
    autoaim_drv_ptr->send_data();
}