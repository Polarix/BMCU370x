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
            speed_set = 10;
        }
        if (motion == filament_motion_pull) // pull
        {
            speed_set = -50;
        }
        if (motion == filament_motion_stop) // just stop
        {
            PID.clear();
            Motion_control_set_PWM(CHx, 0);
            return;
        }
        if (motion == filament_motion_less_pressure) // less pressure
        {
            speed_set = 10;
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

        distance_E = (float)(now_distance - last_distance + cir_E) * AS5600_PI * 7.5 / 4096; // D=7.5mm
        as5600_distance_save[i] = now_distance;

        float speedx = distance_E / T * 1000;
        // T = speed_filter_k / (T + speed_filter_k);
        speed_as5600[i] = speedx; // * (1 - T) + speed_as5600[i] * T; // mm/s
        if (get_filament_motion(i) != need_send_out)
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

bool motor_motion_filamnet_pull_back_to_online_key()
{
    bool wait = false;
    for (int i = 0; i < 4; i++)
    {
        switch (filament_now_position[i])
        {
        case filament_pulling_back:
            RGB_set(i, 0xFF, 0x00, 0xFF);
            if (ONLINE_key_stu_raw[i] == 0)
                MOTOR_CONTROL[i].set_motion(-1, 100);
            else
            {
                MOTOR_CONTROL[i].set_motion(-2, 100);
                if (ONLINE_key_stu[i] != 0)
                    filament_now_position[i] = filament_redetect;
            }
            wait = true;
            break;
        case filament_redetect:
            RGB_set(i, 0xFF, 0xFF, 0x00);
            if (ONLINE_key_stu[i] != 0)
                MOTOR_CONTROL[i].set_motion(1, 100);
            else
            {
                MOTOR_CONTROL[i].set_motion(0, 100);
                filament_now_position[i] = filament_idle;
                set_filament_motion(i, idle);
            }
            wait = true;
            break;
        default:
            break;
        }
    }
    return wait;
}
void motor_motion_switch()
{
    int num = get_now_filament_num();
    if (num == 0xFF)
        return;
    if (get_filament_online(num))
    {

        switch (get_filament_motion(num))
        {
        case need_send_out:
            RGB_set(num, 0x00, 0xFF, 0x00);
            filament_now_position[num] = filament_sending_out;
            if (PULL_key_stu[num] == 0)
            {
                MOTOR_CONTROL[num].set_motion(filament_motion_send, 100);
            }
            else
            {
                MOTOR_CONTROL[num].set_motion(filament_motion_slow_send,100);
            }
            break;
        case need_pull_back:
            RGB_set(num, 0xFF, 0x00, 0xFF);
            filament_now_position[num] = filament_pulling_back;
            MOTOR_CONTROL[num].set_motion(filament_motion_pull, 100);
            break;
        case on_use:
        {
            filament_now_position[num] = filament_using;
            if (PULL_key_stu[num] == 0)
            {
                MOTOR_CONTROL[num].set_motion(filament_motion_less_pressure, 20);
            }

            RGB_set(num, 0xFF, 0xFF, 0xFF);
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
void motor_motion_run(int error)
{

    uint16_t device_type = get_now_BambuBus_device_type();
    if (!error)
    {
        if (device_type == BambuBus_AMS_lite)
        {
            motor_motion_switch();
        }
        else if (device_type == BambuBus_AMS)
        {
            if (!motor_motion_filamnet_pull_back_to_online_key())
            {
                motor_motion_switch();
            }
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

void Motion_control_run(int error)
{
    MC_PULL_key_read();
    MC_ONLINE_key_read();

    AS5600_distance_updata();
    for (int i = 0; i < 4; i++)
    {
        if ((ONLINE_key_stu[i] == 0) && (MC_AS5600.online[i] == true) && (MC_AS5600.magnet_stu[i] != -1))
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
        for (int i = 0; i < 4; i++)
        {
            set_filament_online(i, false);
            if (PULL_key_stu[i] == 0)
            {
                RGB_set(i, 0xFF, 0x00, 0x00);
                if (ONLINE_key_stu[i] == 0)
                {
                    RGB_set(i, 0xFF, 0x00, 0xFF);
                }
            }
            else if (ONLINE_key_stu[i] == 0)
            {
                RGB_set(i, 0x00, 0x00, 0xFF);
            }
            else
            {
                RGB_set(i, 0x00, 0x00, 0x00);
            }
        }
    }
    else
        for (int i = 0; i < 4; i++)
            RGB_set(i, 0x00, 0x00, 0x37);

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
