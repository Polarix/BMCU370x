#include "Motion_control.h"

AS5600_soft_IIC_many MC_AS5600;
uint32_t AS5600_SCL[] = {PA6, PA4, PA2, PA0};
uint32_t AS5600_SDA[] = {PA7, PA5, PA3, PA1};

uint8_t PULL_key_stu[4] = {0, 0, 0, 0};
uint8_t PULL_key_change[4] = {0, 0, 0, 0};
#define PWM_lim 900

struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;
} Motion_control_data_save;

#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)
bool Motion_control_read()
{
    Motion_control_save_struct *ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
    if (ptr->check == 0x40614061)
    {
        memcpy(&Motion_control_data_save, ptr, sizeof(Motion_control_save_struct));
        return true;
    }
    return false;
}
void Motion_control_save()
{
    on_chip_flash_save(&Motion_control_data_save, sizeof(Motion_control_save_struct), Motion_control_save_flash_addr);
}

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

typedef enum _e_filament_motion_
{
    filament_motion_send = 1,
    filament_motion_send_pressure,
    filament_motion_slow_send,
    filament_motion_pull = -1,
    filament_motion_stop = -2,
    filament_motion_no_resistance = 0,
    filament_motion_less_pressure = 100,
    filament_motion_over_pressure = 101,
}filament_motion_enum_t;

class _MOTOR_CONTROL
{
public:
    int m_motion = 0;
    int m_channel_idx = 0;
    uint64_t m_motor_stop_time = 0;
    MOTOR_PID m_PID;
    float pwm_zero = 500;
    float m_dir = 0; /* 0停机，-1反转，1正转，因为参与PID运算，所以做成float */
    int x1 = 0;
    _MOTOR_CONTROL(int _CHx)
    {
        m_channel_idx = _CHx;
        m_motor_stop_time = 0;
        m_motion = 0;
    }
    /* 设置PWM零点值 */
    void set_pwm_zero(float _pwm_zero)
    {
        pwm_zero = _pwm_zero;
    }
    void set_motion(int _motion, uint64_t over_time)
    {
        uint64_t time_now = get_time64();
        m_motor_stop_time = time_now + over_time;
        if (m_motion != _motion)
        {
            m_motion = _motion;
            m_PID.clear();
        }
    }
    int get_motion()
    {
        return m_motion;
    }
    void run(float now_speed)
    {
        uint16_t device_type = get_now_BambuBus_device_type();
        uint64_t time_now = get_time64();
        static uint64_t time_last = 0;
        float speed_set = 0;
        if (time_now >= m_motor_stop_time)
        {
            m_motion = filament_motion_no_resistance;
        }
        if (m_motion == filament_motion_no_resistance)
        {
            m_PID.clear();
            Motion_control_set_PWM(m_channel_idx, 0);
            return;
        }
        if (m_motion == filament_motion_stop) // just stop
        {
            m_PID.clear();
            Motion_control_set_PWM(m_channel_idx, 0);
            return;
        }
        if (m_motion == filament_motion_send) // send
        {
            if (device_type == BambuBus_AMS)
            {
                speed_set = 55; // AMS加速
            } else { // amslite 正常速度
                speed_set = 50;
            }
        }
        if (m_motion == filament_motion_send_pressure)
        {
            speed_set = 20;
        }
        if (m_motion == filament_motion_slow_send) // slowly send
        {
            speed_set = 3;
        }
        if (m_motion == filament_motion_pull) // pull 退料调速
        {
            speed_set = -50; // AMS
        }

        if (m_motion == filament_motion_less_pressure) // less pressure
        {
            // 缓冲时压力不会太小 默认：10
            if (device_type == BambuBus_AMS) // 如果是 BambuBus_AMS 辅助送料速度
            {
                /* 型号：AMS */
                /* 保持慢速辅助送料 阻力大可改成 10-15 */
                speed_set = 10;
            }
            else /* if(device_type == BambuBus_AMS_lite) */
            {
                /* 型号：AMS Lite */ 
                /* 保持慢速辅助送料 */
                speed_set = 10;
            }
        }
        if (m_motion == filament_motion_over_pressure) // over pressure
        {
            speed_set = -10;
        }
        x1 = speed_set;
        float x = m_dir * m_PID.caculate(now_speed - speed_set, (float)(time_now - time_last) / 1000);

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

        Motion_control_set_PWM(m_channel_idx, x);
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
    else // PWM==0
    {
        set1 = 1000;
        set2 = 1000;
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

#define AS5600_PI 3.1415926535897932384626433832795
#define speed_filter_k 100
float speed_as5600[4] = {0, 0, 0, 0};
int32_t as5600_distance_save[4] = {0, 0, 0, 0};
void AS5600_distance_updata()
{
    static uint64_t time_last = 0;
    uint64_t time_now;
    float T;
    do
    {
        time_now = get_time64();
    } while (time_now <= time_last); // T!=0
    T = (float)(time_now - time_last);
    MC_AS5600.updata_angle();
    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false))
        {
            as5600_distance_save[i] = 0;
            speed_as5600[i] = 0;
            continue;
        }

        int32_t cir_E = 0;
        int32_t last_distance = as5600_distance_save[i];
        int32_t now_distance = MC_AS5600.raw_angle[i];
        float distance_E;
        if ((now_distance > 3072) && (last_distance <= 1024))
        {
            cir_E = -4096;
        }
        else if ((now_distance <= 1024) && (last_distance > 3072))
        {
            cir_E = 4096;
        }

        /* BMG挤出轮的送料区直径是7.5mm */
        /* now_distance - last_distance + cir_E，确保当前计算的料丝长度有效，不会溢出 */
        distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096;
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s

        /* 记录料丝增量，单位是米(m)，所以/1000。 */
        add_filament_meters(i, distance_E / 1000);
    }
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

static uint64_t motor_reverse_start_time[4] = {0}; // 记录电机反转开始时间
bool Prepare_For_filament_Pull_Back(uint64_t OUT_TIME)
{
    bool wait = false;
    for (int i = 0; i < 4; i++)
    {
        if (filament_now_position[i] == filament_pulling_back)
        {
            RGB_set(i, 0xFF, 0x00, 0xFF); // 设置RGB灯为紫色
            MOTOR_CONTROL[i].set_motion(filament_motion_pull, 100); // 驱动电机退料
            uint64_t current_time = get_time64();
            if (motor_reverse_start_time[i] == 0) // 如果反转开始时间未记录，则记录当前时间
            {
                motor_reverse_start_time[i] = current_time;
            }
            uint64_t time = current_time - motor_reverse_start_time[i];

            if (time > OUT_TIME) { // 到达停止时间
                MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100); // 停止电机
                MOTOR_CONTROL[i].set_motion(filament_motion_no_resistance, 100); // 设置无阻力模式
                filament_now_position[i] = filament_idle; // 设置当前位置为空闲
                set_filament_motion(i, idle); // 设置当前耗材状态为空闲
                motor_reverse_start_time[i] = 0; // 重置反转开始时间
            }
            wait = true;
        }
    }
    return wait;
}

void motor_motion_switch()
{
    int num = get_now_filament_num();
    uint16_t device_type = get_now_BambuBus_device_type();
    if (num == 0xFF)
        return;
    if (get_filament_online(num))
    {
        switch (get_filament_motion(num))
        {
        case need_send_out:
            RGB_set(num, 0x00, 0xFF, 0x00);
            filament_now_position[num] = filament_sending_out;
            if (device_type == BambuBus_AMS_lite)
            {
                if (PULL_key_stu[num] == 0)
                    MOTOR_CONTROL[num].set_motion(filament_motion_send, 100);
                else
                    MOTOR_CONTROL[num].set_motion(filament_motion_send_pressure, 100);
            }
            else if (device_type == BambuBus_AMS)
            {
                MOTOR_CONTROL[num].set_motion(filament_motion_send, 100);
            }
            break;
        case need_pull_back:
            RGB_set(num, 0xFF, 0x00, 0xFF);
            filament_now_position[num] = filament_pulling_back;
            // MOTOR_CONTROL[num].set_motion(filament_motion_pull, 100);
            break;
        case on_use:
        {
            static uint64_t time_end = 0;
            uint64_t time_now = get_time64();
            if (filament_now_position[num] == filament_sending_out)
            {
                filament_now_position[num] = filament_using;

                time_end = time_now + 3000;
            }
            else if (filament_now_position[num] == filament_using) // 已经进入使用状态,即打印机已经检测到耗材丝。
            {
                if (PULL_key_stu[num] == 0) // 如果触发缓冲，则缓冲状态。
                    MOTOR_CONTROL[num].set_motion(filament_motion_less_pressure, 20); // 缓冲状态。
                else if (time_now < time_end) // 如果是刚进料且在前三秒，使用慢速送料，避免没被工具头咬合。
                    MOTOR_CONTROL[num].set_motion(filament_motion_slow_send, 20); // 缓慢
                else // 已经超过3秒，如果未触发缓冲则紧急刹车。
                    MOTOR_CONTROL[num].set_motion(filament_motion_stop, 20);
            }
            RGB_set(num, 0xFF, 0xFF, 0xFF); // 设置RGB灯为白色，进入使用状态。
            break;
        }
        case idle:
            filament_now_position[num] = filament_idle;
            MOTOR_CONTROL[num].set_motion(filament_motion_no_resistance, 100);
            RGB_set(num, 0x00, 0x00, 0x37);
            break;
        }
    }
}

// 根据AMS模拟器的信息，来调度电机
void motor_motion_run(int error)
{   // 退料时间
    uint64_t A1X_OUT_TIME = 2300;
    uint64_t P1X_OUT_TIME = 3100;
    uint64_t OUT_TIME = 8000;
    uint16_t device_type = get_now_BambuBus_device_type();

    if (!error)
    {
        // 根据设备类型执行不同的电机控制逻辑
        if (device_type == BambuBus_AMS_lite)
        {
            OUT_TIME = A1X_OUT_TIME;
        }
        else if(device_type == BambuBus_AMS)
        {
            OUT_TIME = P1X_OUT_TIME;
        } // 执行
        if (!Prepare_For_filament_Pull_Back(OUT_TIME))
        { // 如果 wait 返回 false，不用等待。
            motor_motion_switch(); // 完成退料
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
    }

    for (int i = 0; i < 4; i++)
    {
        if (!get_filament_online(i))
            MOTOR_CONTROL[i].set_motion(filament_motion_stop, 100);
        MOTOR_CONTROL[i].run(speed_as5600[i]);
    }
}

void motion_control_ticks_handler(int error)
{
    /* 更新缓冲器状态 */    
    MC_PULL_key_read();
    /* 更新料丝状态 */
    MC_ONLINE_key_read();
    /* 更新送料齿轮状态 */
    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if ((ONLINE_key_stu[i] == 0))
        {
            set_filament_online(i, true);
        }
        else if ((filament_now_position[i] != filament_redetect) && (filament_now_position[i] != filament_pulling_back))
        {
            set_filament_online(i, false);
        }
    }
    if (error)
    {
        /* error值非零（通过代码分析大概是-1），BMCU处于离线状态。 */
        for (int i = 0; i < 4; i++)
        {
            /* 清除料丝存在状态 */
            set_filament_online(i, false);
            if (PULL_key_stu[i] == 0)
            {
                /* 缓冲器张紧状态 */
                RGB_set(i, 0xFF, 0x00, 0x00);
                if (ONLINE_key_stu[i] == 0)
                {
                    RGB_set(i, 0xFF, 0x00, 0xFF);
                }
            }
            else if (ONLINE_key_stu[i] == 0)
            {
                /* 缓冲器松弛状态 */
                RGB_set(i, 0x00, 0x00, 0xFF);
            }
            else
            {
                /* 理论上不会走到这个分支 */
                RGB_set(i, 0x00, 0x00, 0x00);
            }
        }
    }
    else
    {
        /* BMCU在线状态，更新各通道指示灯颜色 */
        for (int i = 0; i < 4; i++)
        {
            RGB_set(i, 0x00, 0x00, 0x37);
        }
    }
    motor_motion_run(error);

    for (int i = 0; i < 4; i++)
    {
        if ((MC_AS5600.online[i] == false) || (MC_AS5600.magnet_stu[i] == -1)) // AS5600 error
        {
            RGB_set(i, 0xFF, 0x00, 0x00);
        }
    }
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

int M5600_angle_dis(int16_t angle1, int16_t angle2)
{

    int cir_E = angle1 - angle2;
    if ((angle1 > 3072) && (angle2 <= 1024))
    {
        cir_E = -4096;
    }
    else if ((angle1 <= 1024) && (angle2 > 3072))
    {
        cir_E = 4096;
    }
    return cir_E;
}
void MOTOR_get_dir()
{
    int dir[4] = {0, 0, 0, 0};
    bool done = false;
    bool have_data = Motion_control_read();
    if (!have_data)
    {
        for (int index = 0; index < 4; index++)
        {
            Motion_control_data_save.Motion_control_dir[index] = 0;
        }
    }
    MC_AS5600.updata_angle(); // read as5600 once

    int16_t last_angle[4];
    for (int index = 0; index < 4; index++)
    {
        last_angle[index] = MC_AS5600.raw_angle[index];                  // init angle
        dir[index] = Motion_control_data_save.Motion_control_dir[index]; // init dir data
    }
    bool need_test = false; // 是否需要检测
    bool need_save = false; // 是否需要更新状态
    for (int index = 0; index < 4; index++)
    {
        if ((MC_AS5600.online[index] == true)) // 有5600，说明通道在线
        {
            if (Motion_control_data_save.Motion_control_dir[index] == 0) // 之前测试结果为0，需要测试
            {
                Motion_control_set_PWM(index, 1000); // 打开电机
                need_test = true;                    // 设置需要测试
                need_save = true;                    // 有状态更新
            }
        }
        else
        {
            dir[index] = 0;   // 通道不在线，清空它的方向数据
            need_save = true; // 有状态更新
        }
    }
    int i = 0;
    while (done == false)
    {
        done = true;
        
        delay(10);//间隔10ms检测一次
        MC_AS5600.updata_angle();//更新角度数据

        if (i++ > 200)//超过2s无响应
        {
            for (int index = 0; index < 4; index++)
            {
                Motion_control_set_PWM(index, 0); // 停止
                Motion_control_data_save.Motion_control_dir[index] = 0;//方向设为0
            }
            break;//跳出循环
        }
        for (int index = 0; index < 4; index++)//遍历
        {
            if ((MC_AS5600.online[index] == true) && (Motion_control_data_save.Motion_control_dir[index] == 0)) // 对于新的通道
            {
                int angle_dis = M5600_angle_dis(MC_AS5600.raw_angle[index], last_angle[index]);
                if (abs(angle_dis) > 163) // 移动超过1mm
                {
                    Motion_control_set_PWM(index, 0); // 停止
                    if (angle_dis < 0)
                    {
                        dir[index] = 1;
                    }
                    else
                    {
                        dir[index] = -1;
                    }
                }
                else
                {
                    done = false;//没有移动。继续等待
                }
            }
        }
        
    }
    for (int index = 0; index < 4; index++)//遍历四个电机
    {
        Motion_control_data_save.Motion_control_dir[index] = dir[index];//数据复制
    }
    if (need_save)//如果需要保存数据
    {
        Motion_control_save();//数据保存
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
    MOTOR_get_dir();
    for (int index = 0; index < 4; index++)
    {
        Motion_control_set_PWM(index, 0);
        MOTOR_CONTROL[index].set_pwm_zero(500);
        MOTOR_CONTROL[index].m_dir = Motion_control_data_save.Motion_control_dir[index];
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
