#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PA6, PA4, PA2, PA0};
uint32_t AS5600_SDA[] = {PA7, PA5, PA3, PA1};

uint8_t PULL_key_stu[4] = {0, 0, 0, 0};
uint8_t PULL_key_change[4] = {0, 0, 0, 0};
#define PWM_lim 1000

class MOTOR_PID
{
public:
    float P = 2;
    float I = 10;
    float D = 0;
    float I_save = 0;
    float E_last = 0;
    float pid_MAX = PWM_lim;
    float pid_MIN = -PWM_lim;
    float pid_range = (pid_MAX - pid_MIN) / 2;
    void init(float P_set, float I_set)
    {
        P = P_set;
        I = I_set;
        I_save = 0;
    }
    float caculate(float E, float time_E)
    {
        I_save += I * E * time_E;
        if (I_save > pid_range / 2) // 对I限幅
            I_save = pid_range / 2;
        if (I_save < -pid_range / 2)
            I_save = -pid_range / 2;

        float ouput_buf = P * (E + (I_save) + D * (E - E_last) / time_E);
        if (ouput_buf > pid_MAX)
            ouput_buf = pid_MAX;
        if (ouput_buf < pid_MIN)
            ouput_buf = pid_MIN;

        E_last = E;
        return ouput_buf;
    }
    void clear()
    {
        I_save = 0;
        E_last = 0;
    }
};

enum filament_motion_enum
{
    filament_motion_send = 1,
    filament_motion_slow_send = 2,
    filament_motion_pull = -1,
    filament_motion_stop = -2,
    filament_motion_no_resistance = 0,
    filament_motion_less_pressure = 100,
    filament_motion_over_pressure = 101,

    filament_motion_slow_pull = 3,
    filament_motion_slow_send_send = -3,
};


class _MOTOR_CONTROL
{
public:
    int motion = 0;
    int CHx = 0;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID;
    float pwm_zero = 500;
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        CHx = _CHx;
        motor_stop_time = 0;
        motion = 0;
    }

    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(int _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        motor_stop_time = time_now + over_time;
        motion = _motion;
    }
    int get_motion()
    {
        return motion;
    }
    void run(float now_speed)
    {
        uint16_t device_type = get_now_BambuBus_device_type();
        uint64_t time_now = get_time64();
        static uint64_t time_last = 0;
        float speed_set = 0;
        if (time_now >= motor_stop_time)
        {
            motion = filament_motion_no_resistance;
        }
        if (motion == filament_motion_no_resistance)
        {
            PID.clear();

            Motion_control_set_PWM(CHx, 0);
            return;
        }
        if (motion == filament_motion_send) // send
        {
            speed_set = 50;
        }

        if (motion == filament_motion_slow_send) // slowly send
        {   
            if (device_type == BambuBus_AMS) // 如果是 BambuBus_AMS 缓冲时，速度不会太慢。
            {
                speed_set = 35;
            } else { // amslite 加速
                speed_set = 15;
            }
        }

        if (motion == filament_motion_slow_send_send ) 
        {
            speed_set = 15;
        }
        if(motion == filament_motion_slow_pull)
        {
            speed_set = -15;
        }

        if (motion == filament_motion_pull) // pull
        {
            if (device_type == BambuBus_AMS) // 如果是 BambuBus_AMS 退料速度 反转50.
            {
                speed_set = -50;
            } else { // amslite 退料减速 控制距离
                speed_set = -35;
            }
        }
        if (motion == filament_motion_stop) // just stop
        {
            PID.clear();
            Motion_control_set_PWM(CHx, 0);
            return;
        }
        if (motion == filament_motion_less_pressure) // less pressure
        {
            // 缓冲时压力不会太小 默认：10
            if (device_type == BambuBus_AMS) // 如果是 BambuBus_AMS 送料速度
            {
                speed_set = 15;
            } else { // amslite 退料减速 控制距离
                speed_set = 10;
            }
        }

        if (motion == filament_motion_over_pressure) // over pressure
        {
            speed_set = -10;
        }
        x1 = speed_set;
        float x = PID.caculate(now_speed - speed_set, (float)(time_now - time_last) / 1000);

        if (x > 1)
            x += pwm_zero;
        else if (x < 1)
            x -= pwm_zero;
        else
            x = 0;

        if (x > PWM_lim)
        {
            x = PWM_lim;
        }
        if (x < -PWM_lim)
        {
            x = -PWM_lim;
        }

        Motion_control_set_PWM(CHx, x);
        time_last = time_now;
    }
};
_MOTOR_CONTROL MOTOR_CONTROL[4] = {_MOTOR_CONTROL(0), _MOTOR_CONTROL(1), _MOTOR_CONTROL(2), _MOTOR_CONTROL(3)};
void MC_PULL_key_read(void)
{
    PULL_key_stu[3] = digitalRead(PB12);
    PULL_key_stu[2] = digitalRead(PB13);
    PULL_key_stu[1] = digitalRead(PB14);
    PULL_key_stu[0] = digitalRead(PB15);
}
void MC_PULL_key_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    MC_PULL_key_read();
}

uint8_t ONLINE_key_stu[4] = {0, 0, 0, 0};
uint8_t ONLINE_key_stu_raw[4] = {0, 0, 0, 0};
uint64_t ONLINE_key_stu_count[4] = {0, 0, 0, 0};
void MC_ONLINE_key_read(void)
{
    uint64_t time_now = get_time64();
    uint64_t time_set = time_now + 1000;
    ONLINE_key_stu_raw[0] = digitalRead(PD0);
    ONLINE_key_stu_raw[1] = digitalRead(PC15);
    ONLINE_key_stu_raw[2] = digitalRead(PC14);
    ONLINE_key_stu_raw[3] = digitalRead(PC13);

    for (int i = 0; i < 4; i++)
    {
        if (ONLINE_key_stu[i] == ONLINE_key_stu_raw[i])
            ONLINE_key_stu_count[i] = time_set;
        else if (ONLINE_key_stu_count[i] < time_now)
        {
            ONLINE_key_stu[i] = ONLINE_key_stu_raw[i];
        }
    }
}

void MC_ONLINE_key_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    MC_ONLINE_key_read();
}

void Motion_control_set_PWM(uint8_t CHx, int PWM)
{
    uint16_t set1 = 0, set2 = 0;
    if (PWM > 0)
    {
        set1 = PWM;
    }
    else if (PWM < 0)
    {
        set2 = -PWM;
    }
    switch (CHx)
    {
    case 3:
        TIM_SetCompare1(TIM2, set1);
        TIM_SetCompare2(TIM2, set2);
        break;
    case 2:
        TIM_SetCompare1(TIM3, set1);
        TIM_SetCompare2(TIM3, set2);
        break;
    case 1:
        TIM_SetCompare1(TIM4, set1);
        TIM_SetCompare2(TIM4, set2);
        break;
    case 0:
        TIM_SetCompare3(TIM4, set1);
        TIM_SetCompare4(TIM4, set2);
        break;
    }
}

// 定义圆周率常量，用于后续的距离计算
#define AS5600_PI 3.1415926535897932384626433832795

// 定义速度滤波系数，用于平滑速度计算（未使用）
#define speed_filter_k 100

// 存储每个通道的当前速度（单位：mm/s）
float speed_as5600[4] = {0, 0, 0, 0};

// 存储每个通道上次记录的角度位置
int32_t as5600_distance_save[4] = {0, 0, 0, 0};

// 更新AS5600传感器的距离信息，并计算速度
void AS5600_distance_updata()
{
    // 静态变量保存上次的时间戳，确保时间差不为零
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;

    // 确保time_now大于time_last，避免时间差为零的情况
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T != 0

    // 计算时间差（单位：ms）
    T = (float)(time_now - time_last) / 1000.0;

    // 更新编码器角度信息
    MC_AS5600.updata_angle();

    // 遍历所有通道（假设最多4个）
    for (int i = 0; i < 4; i++)
    {
        // 如果当前通道的编码器不在线，则重置距离和速度为0
        if (!MC_AS5600.online[i])
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        // 初始化圈数补偿值
        int32_t cir_E = 0;

        // 获取上次和当前的角度位置
        int32_t last_distance = as5600_distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];

        // 处理角度跨越0度的情况
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096; // 从负半圈到正半圈
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096; // 从正半圈到负半圈
        }

        // 计算实际移动的距离（单位：mm），D=7.5mm为编码器轮子直径
        float distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096;

        // 更新上次记录的角度位置
        as5600_distance_save[i] = now_distance;

        // 计算当前速度（单位：mm/s）
        float speedx = distance_E / T;

        // 更新速度数组
        speed_as5600[i] = speedx;

        // 如果当前耗材不是需要送出状态，则更新累计耗材长度
        if (get_filament_motion(i) != need_send_out)
        {
            add_filament_meters(i, distance_E / 1000);
        }
    }

    // 更新上次的时间戳
    time_last = time_now;
}

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
};
int filament_now_position[4];
bool wait = false;

// set_motion （模式.结束时间）
// -1 = 退出 1抽入 0停止
// 退料时间
#define A1X_OUT_TIME 2500;
#define P1S_OUT_TIME 3500;
static uint64_t motor_reverse_start_time[4] = {0}; // 记录电机反转开始时间

bool Prepare_For_filament_Pull_Back(int type)
{
    bool wait = false;
    int OUT_TIME = 0;
    if (type == 1)
    {
        OUT_TIME = A1X_OUT_TIME;
    } else if (type == 0) {
        OUT_TIME = P1S_OUT_TIME;
    }

    for (int i = 0; i < 4; i++)
    {
        switch (filament_now_position[i])
        {
            case filament_pulling_back:
                RGB_set(i, 0xFF, 0x00, 0xFF); // 设置RGB灯为紫色
                uint64_t current_time = get_time64();
                if (motor_reverse_start_time[i] == 0) // 如果反转开始时间未记录，则记录当前时间
                {
                    motor_reverse_start_time[i] = current_time;
                }
                uint64_t time = current_time - motor_reverse_start_time[i];

                if (time > OUT_TIME) { // 到达停止时间
                    MOTOR_CONTROL[i].set_motion(-2, 100); // 停止电机
                    MOTOR_CONTROL[i].set_motion(0, 100); // 设置无阻力模式
                    filament_now_position[i] = filament_idle; // 设置当前位置为空闲
                    set_filament_motion(i, idle); // 设置当前耗材状态为空闲
                    motor_reverse_start_time[i] = 0; // 重置反转开始时间
                } else {
                    MOTOR_CONTROL[i].set_motion(-1, 100); // 驱动电机退料
                }
                wait = true;
                break;
            // 默认情况下，不执行任何操作
            default:
                break;
        }
    }
    return wait;
}

// motor_motion_switch函数根据当前耗材的状态切换电机的动作模式
void motor_motion_switch()
{
    // 获取当前使用的耗材编号
    int num = get_now_filament_num();
    
    // 如果没有有效的耗材编号（0xFF表示无效），直接返回
    if (num == 0xFF)
        return;

    // 检查当前耗材是否在线
    if (get_filament_online(num))
    {
        // 根据当前耗材的运动状态执行不同的动作
        switch (get_filament_motion(num))
        {
            case need_send_out: // 需要送出耗材
                RGB_set(num, 0x00, 0xFF, 0x00); // 设置LED为绿色
                filament_now_position[num] = filament_sending_out; // 更新耗材位置状态为正在送出
                
                // 根据拔出键状态设置不同的送出速度
                if (PULL_key_stu[num] == 0) // 如果拔出键未按下
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_send, 100); // 快速送出
                }
                else // 如果拔出键已按下
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_slow_send, 100); // 缓慢送出
                }
                break;

            case need_pull_back: // 需要退回耗材
                RGB_set(num, 0xFF, 0x00, 0xFF); // 设置LED为紫色
                filament_now_position[num] = filament_pulling_back; // 更新耗材位置状态为正在退回
                // MOTOR_CONTROL[num].set_motion(filament_motion_pull, 100); // 退回耗材
                break;

            case on_use: // 耗材正在使用
            {
                filament_now_position[num] = filament_using; // 更新耗材位置状态为正在使用
                
                // 如果拔出键未按下，设置为低压力模式
                if (PULL_key_stu[num] == 0)
                {
                    MOTOR_CONTROL[num].set_motion(filament_motion_less_pressure, 20); // 设置低压力模式
                }

                RGB_set(num, 0xFF, 0xFF, 0xFF); // 设置LED为白色
                break;
            }

            case idle: // 空闲状态
                filament_now_position[num] = filament_idle; // 更新耗材位置状态为空闲
                MOTOR_CONTROL[num].set_motion(filament_motion_no_resistance, 100); // 设置为无阻力模式
                RGB_set(num, 0x00, 0x00, 0x37); // 设置LED为淡蓝色
                break;
        }
    }
}

// motor_motion_run函数根据错误状态和设备类型控制电机动作
void motor_motion_run(int error)
{
    uint16_t device_type = get_now_BambuBus_device_type();
    // 如果没有错误发生
    if (!error)
    {
        // 根据设备类型执行不同的电机控制逻辑
        if (device_type == BambuBus_AMS_lite)
        {
            // 对于AMS_lite设备，直接调用motor_motion_switch切换电机状态
            if (!Prepare_For_filament_Pull_Back(1)) // 如果 wait 返回 false，不用等待。
            {
            motor_motion_switch(); // 完成退料
            }
        }
        else if (device_type == BambuBus_AMS)
        {
            // 对于AMS设备，特殊退料处理
            if (!Prepare_For_filament_Pull_Back(0)) // 如果 wait 返回 false，不用等待。
            {
                motor_motion_switch(); // 完成退料
            }
        }
    }
    else
    {
        // 如果有错误发生，停止所有电机的动作
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
    }

    // 遍历所有电机通道
    for (int i = 0; i < 4; i++)
    {
        // 如果耗材不在在线状态，则停止该通道的电机动作
        if (!get_filament_online(i))
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
        
        // 根据当前速度调整电机PWM输出
        MOTOR_CONTROL[i].run(speed_as5600[i]);
    }
}

// Motion_control_run函数负责整体运动控制流程，包括按键读取、距离更新及电机运行
void Motion_control_run(int error)
{
    // 读取拔出按键状态
    MC_PULL_key_read();
    
    // 读取在线检测按键状态
    MC_ONLINE_key_read();

    // 更新AS5600传感器的距离信息
    AS5600_distance_updata();
    
    // 遍历所有通道更新耗材在线状态
    for (int i = 0; i < 4; i++)
    {
        // 如果耗材在线 且AS5600传感器在线 且磁铁状态正常，则设置耗材为在线
        if ((ONLINE_key_stu[i] == 0) && (MC_AS5600.online[i] == true) && (MC_AS5600.magnet_stu[i] != -1))
        {
            set_filament_online(i, true);
        }
        // 否则，如果不是处于重新检测 或 退料状态，则设置耗材为离线
        else if ((filament_now_position[i] != filament_redetect) && (filament_now_position[i] != filament_pulling_back))
        {
            set_filament_online(i, false);
        }
    }
    
    // 如果存在错误
    if (error)
    {
        // 设置所有耗材为离线状态，并根据不同条件设置LED颜色
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            // 缓冲未触发时
            if (PULL_key_stu[i] == 0)
            {
                RGB_set(i, 0xFF, 0x00, 0x00); // 红色
                
                // 如果检测到耗材，则设置为紫色
                if (ONLINE_key_stu[i] == 0)
                {
                    RGB_set(i, 0xFF, 0x00, 0xFF);
                }
            }
            // 如果检测到耗材并且触发缓冲，则设置为蓝色
            else if (ONLINE_key_stu[i] == 0)
            {
                RGB_set(i, 0x00, 0x00, 0xFF);
            }
            // 其他情况设置为黑色
            else
            {
                RGB_set(i, 0x00, 0x00, 0x00);
            }
            /*
            static unsigned long error_start_time[4] = {0}; // 记录错误开始时间
            static unsigned long loop_start_time[4] = {0};   // 记录循环开始时间
            static int loop_state[4] = {0};                 // 记录当前循环状态

            unsigned long current_time = millis();
            // 离线10秒后进入debug电机模式。
            if (error_start_time[i] == 0)
            {
                error_start_time[i] = current_time;
            }
            if (current_time - error_start_time[i] >= 10000) // 10秒
            {
            // 如果循环开始时间未记录，则记录当前时间
            if (loop_start_time[i] == 0)
            {
                loop_start_time[i] = current_time;
            }

            // 计算当前时间与循环开始时间的差值
            unsigned long loop_time = current_time - loop_start_time[i];

            // 根据循环时间执行不同的电机操作
            if (loop_time < 3000) // 3秒正转
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_send, 100);
                loop_state[i] = 1;
            }
            else if (loop_time >= 3000 && loop_time < 6000) // 3秒停止
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
                loop_state[i] = 2;
            }
            else if (loop_time >= 6000 && loop_time < 9000) // 3秒反转
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_pull, 100);
                loop_state[i] = 3;
            }
            else if (loop_time >= 9000 && loop_time < 12000) // 3秒停止
            {
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
                loop_state[i] = 4;
            }
            else
            {
                loop_start_time[i] = current_time; // 重置循环开始时间
            }
        } */
        }
    }
    else
    {
        // 如果无错误，则将所有LED设置为淡蓝色
        for (int i = 0; i < 4; i++)
            RGB_set(i, 0x00, 0x00, 0x37);
    }
    // 最后调用motor_motion_run函数执行具体的电机控制逻辑
    motor_motion_run(error);
}


void MC_PWM_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 |
                                  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 开启复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); // 开启TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); // 开启TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // 开启TIM4时钟

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    // 定时器基础配置
    TIM_TimeBaseStructure.TIM_Period = 999;  // 周期（x+1）
    TIM_TimeBaseStructure.TIM_Prescaler = 1; // 预分频（x+1）
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    // PWM模式配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; // 占空比
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA15
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PB3
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PB4
    TIM_OC2Init(TIM3, &TIM_OCInitStructure); // PB5
    TIM_OC1Init(TIM4, &TIM_OCInitStructure); // PB6
    TIM_OC2Init(TIM4, &TIM_OCInitStructure); // PB7
    TIM_OC3Init(TIM4, &TIM_OCInitStructure); // PB8
    TIM_OC4Init(TIM4, &TIM_OCInitStructure); // PB9

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);    // TIM2完全映射-CH1-PA15/CH2-PB3
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE); // TIM3部分映射-CH1-PB4/CH2-PB5
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, DISABLE);       // TIM4不映射-CH1-PB6/CH2-PB7/CH3-PB8/CH4-PB9

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM4, ENABLE);
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

void MOTOR_get_pwm_zero()
{
    float pwm_zero[4] = {0, 0, 0, 0};
    MC_AS5600.updata_angle();

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];
    }
    for (int pwm = 300; pwm < 1000; pwm += 10)
    {
        MC_AS5600.updata_angle();
        for (int index = 0; index < 4; index++)
        {

            if (pwm_zero[index] == 0)
            {
                if (abs(MC_AS5600.raw_angle[index] - last_angle[index]) > 50)
                {
                    pwm_zero[index] = pwm;
                    pwm_zero[index] *= 0.90;
                    Motion_control_set_PWM(index, 0);
                }
                else if ((MC_AS5600.online[index] == true))
                {
                    Motion_control_set_PWM(index, -pwm);
                }
            }
            else
            {
                Motion_control_set_PWM(index, 0);
            }
        }
        delay(100);
    }
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(pwm_zero[index]);
    }
}

void MOTOR_init()
{

    MC_PWM_init();
    MC_AS5600.init(AS5600_SCL, AS5600_SDA, 4);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        as5600_distance_save[i] = MC_AS5600.raw_angle[i];
    }
    // MOTOR_get_pwm_zero();
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
    }
}

void Motion_control_init()
{

    MC_PULL_key_init();
    MC_ONLINE_key_init();

    MOTOR_init();
    for (int i = 0; i < 4; i++)
    {
        filament_now_position[i] = filament_idle;
    }
}
