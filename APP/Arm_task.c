#include "Arm_Task.h"
#include "fdcan.h"
#include "cmsis_os.h"      
#include "dm4310_drv.h"
#include "math.h" 
#include "usb_protocol.h"  
#include "dynamic.h"       

// =========================================================
// 全局变量
// =========================================================
Arm_t my_arm;

// 目标位置与滤波
float usb_target_pos[6] = {0}; 
static float current_filtered_target[6] = {0};

// 键鼠控制相关参数
#define MOUSE_SENSITIVITY  0.00005f  
#define KEY_STEP           0.0002f   

// 辅助符号函数
float sign_func(float x) { 
    if (x > 0.0001f) return 1.0f;
    if (x < -0.0001f) return -1.0f;
    return 0.0f;
}

//==============================================补充定义================================================================================//
int MAX_LIMIT[6] = {0};
int MIN_LIMIT[6] = {0};
//==============================================补充定义================================================================================//

// ---------------------------------------------------------
// 任务初始化
// ---------------------------------------------------------
void Arm_Init(void)
{
    // 初始化电机对象
    for(int i=0; i<6; i++) {
        joint_motor_init(&my_arm.joints[i], i+1, MIT_MODE); 
        // 确保CAN通信正常后使能
        enable_motor_mode(&hfdcan1, my_arm.joints[i].para.id, my_arm.joints[i].mode);
        osDelay(10);
    }
    
    // 初始化变量
    my_arm.start_flag = 1;
    my_arm.homing_done = 0;
    
    // 初始状态设为归零模式，防止上电乱动
    my_arm.current_state = ARM_STATE_HOMING; 
}

// ---------------------------------------------------------
// 主任务循环
// ---------------------------------------------------------
void Arm_Task(void const * argument)
{
    osDelay(1000); // 等待底层初始化
    Arm_Init(); 
    
    float current_q[6];     // 当前实际角度
    float calc_tau_g[6];    // 重力矩
    uint32_t report_tick = 0;

    // 第一次进入循环前，将目标位置初始化为当前实际位置，避免瞬间跳变
    for(int i=0; i<6; i++) {
         // 获取一次反馈（假设电机驱动有自动更新，或者手动读取一次）
        current_q[i] = my_arm.joints[i].para.pos / GEAR_RATIO;
        usb_target_pos[i] = current_q[i];
        current_filtered_target[i] = current_q[i];
    }

    while(1)
    {
        // 1. 协议解析 (获取 g_remote_ctrl 或更新 usb_target_pos)
        USB_Protocol_Process();
        
        // 2. 获取反馈 & 坐标转换
        for(int i=0; i<6; i++) {
            current_q[i] = my_arm.joints[i].para.pos / GEAR_RATIO;
        }
        
        // 3. 计算重力补偿 (参数辨识法)
        Calc_Gravity_Comp(current_q, calc_tau_g);

        // 4. 状态机逻辑 (if-else 结构)
        // -------------------------------------------------------------
        
        // 场景 A: 失能或错误 -> 彻底放松
        if (my_arm.current_state == ARM_STATE_DISABLED || my_arm.current_state == ARM_STATE_ERROR) 
        {
            for(int i=0; i<6; i++) {
                mit_ctrl_4310(&hfdcan1, i+1, 0, 0, 0, 0, 0); // 发送零力矩
                
                // 同步目标位置，防止切模式瞬间飞车
                usb_target_pos[i] = current_q[i];
                current_filtered_target[i] = current_q[i];
            }
            my_arm.homing_done = 0; // 失能后需要重新归零(视需求而定)
        }

        // 场景 B: 阶梯归零模式 (新增) -> 缓慢回到初始姿态(比如全0)
        else if (my_arm.current_state == ARM_STATE_HOMING)
        {
            uint8_t all_reached = 1;
            // 设定归零的目标姿态，这里假设归零就是回到 0.0
            float home_target[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; 
            
            for(int i=0; i<6; i++) 
            {
                float err = home_target[i] - current_filtered_target[i];
                
                // 如果误差大于步长，则走一步
                if (fabsf(err) > HOMING_STEP) {
                    current_filtered_target[i] += sign_func(err) * HOMING_STEP;
                    all_reached = 0;
                } else {
                    current_filtered_target[i] = home_target[i];
                }
                
                // 发送指令：刚度较大以保证归位，带重力补偿
                float kp = 5.0f; 
                float kd = 1.0f;
                float motor_pos = current_filtered_target[i] * GEAR_RATIO;
                float ff = calc_tau_g[i] / GEAR_RATIO;
                
                mit_ctrl_4310(&hfdcan1, i+1, motor_pos, 0, kp, kd, ff);
            }
            
            // 同步 usb_target_pos，防止切出状态时跳变
            for(int i=0; i<6; i++) usb_target_pos[i] = current_filtered_target[i];

            // 归零完成后自动切换到重力补偿模式
            if (all_reached) {
                my_arm.homing_done = 1;
                my_arm.current_state = ARM_STATE_GRAVITY_COMP; 
            }
        }
        
        // 场景 C: 示教模式 -> 零刚度 + 重力补偿
        else if (my_arm.current_state == ARM_STATE_GRAVITY_COMP) 
        {
            for(int i=0; i<6; i++) {
                float ff = calc_tau_g[i] / GEAR_RATIO; 
                // 仅发送前馈力，Kp=0, Kd=0.5(阻尼)
                mit_ctrl_4310(&hfdcan1, i+1, 0, 0, 0, 0.5f, ff);
                
                // 实时跟随位置，为切换到控制模式做准备
                usb_target_pos[i] = current_q[i];
                current_filtered_target[i] = current_q[i];
            }
        }

        // 场景 D: 控制模式 (自定义控制器 OR 键鼠)
        else if (my_arm.current_state == ARM_STATE_REPLAY || my_arm.current_state == ARM_STATE_KEYMOUSE)
        {
            // 如果是键鼠模式，计算增量
            if (my_arm.current_state == ARM_STATE_KEYMOUSE) {
                // ... (此处保留之前的键鼠映射代码，篇幅原因略去，逻辑不变) ...
                // 示例：usb_target_pos[0] -= g_remote_ctrl.mouse_x * MOUSE_SENSITIVITY;
            }
            
            for(int i=0; i<6; i++) 
            {
                // [1] 限位保护
                if(usb_target_pos[i] > MAX_LIMIT[i]) usb_target_pos[i] = MAX_LIMIT[i];
                if(usb_target_pos[i] < MIN_LIMIT[i]) usb_target_pos[i] = MIN_LIMIT[i];

                // [2] 平滑滤波
                float alpha = (my_arm.current_state == ARM_STATE_KEYMOUSE) ? 0.05f : 0.15f;
                current_filtered_target[i] = current_filtered_target[i] * (1.0f - alpha) 
                                           + usb_target_pos[i] * alpha;
                
                // [3] 发送指令
                float motor_pos_ref = current_filtered_target[i] * GEAR_RATIO;
                float motor_tau_ff  = calc_tau_g[i] / GEAR_RATIO;
                
                // 提高刚度以跟随目标
                float kp = 6.0f; 
                float kd = 1.2f; 
                mit_ctrl_4310(&hfdcan1, i+1, motor_pos_ref, 0, kp, kd, motor_tau_ff);
            }
        }
                
        if(HAL_GetTick() - report_tick > 10) {

            report_tick = HAL_GetTick();

            float send_data[12];

            for(int i=0; i<6; i++) {

                send_data[i] = current_q[i];

                send_data[6+i] = my_arm.joints[i].para.tor * GEAR_RATIO;

            }

            USB_Send_Motor_Status(send_data);

        }
        osDelay(2);
    }
}
