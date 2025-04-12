#include <Arduino.h>
#include "main.h"

#include "BambuBus.h"

extern void debug_send_run();
#include "WS2812.h"
WS2812_class SYS_RGB;
WS2812_class RGBOUT[4];

void RGB_init()
{
    SYS_RGB.init(1, PD1);
    RGBOUT[3].init(1, PB0);
    RGBOUT[2].init(1, PB1);
    RGBOUT[1].init(1, PA8);
    RGBOUT[0].init(1, PA11);
}
void RGB_update()
{
    SYS_RGB.updata();
    RGBOUT[0].updata();
    RGBOUT[1].updata();
    RGBOUT[2].updata();
    RGBOUT[3].updata();
}
void RGB_set(uint8_t CHx, uint8_t R, uint8_t G, uint8_t B)
{
    RGBOUT[CHx].set_RGB(R, G, B, 0);
}
extern void bambu_bus_init_usart();
extern void bambu_bus_send_data(const uint8_t *data, uint16_t length);

void setup()
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_PD01, ENABLE);
    RGB_init();
    SYS_RGB.set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[0].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[1].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[2].set_RGB(0x00, 0x00, 0x00, 0);
    RGBOUT[3].set_RGB(0x00, 0x00, 0x00, 0);
    RGB_update();
    
    bambu_bus_init();
    DEBUG_INIT();
    Motion_control_init();
    delay(1);
}

// extern double distance_count;
uint8_t R = 0, G = 0, B = 0;
uint8_t T_to_tangle(uint32_t time)
{
    time = time % 256;
    if (time > 128)
        return 512 - time * 2;
    else
        return time * 2;
}

void loop()
{
    bambu_bus_package_type_t stu = bambu_bus_ticks_handler();
    // int stu =-1;
    static int error = 0;
    bool motion_can_run = false;
    bambu_bus_device_type_t device_type = bambu_bus_get_device_type();
    if (stu != BambuBus_package_NONE) // have data/offline
    {
        motion_can_run = true;
        if (stu == BambuBus_package_ERROR) // offline
        {
            error = -1;
            SYS_RGB.set_RGB(0x30, 0x00, 0x00, 0);
            RGB_update();
        }
        else // have data
        {
            error = 0;
            if (stu == BambuBus_package_heartbeat)
            {
                if(device_type==BambuBus_AMS_lite)
                    SYS_RGB.set_RGB(0x00, 0x00, 0x10, 0);
                else if(device_type==BambuBus_AMS)
                    SYS_RGB.set_RGB(0x10, 0x10, 0x00, 0);
                RGB_update();
            }
        }
    }
    else//wait for data
    {

    }
    if (motion_can_run)
    {
        Motion_control_run(error);    
    }
}
/*
extern void set_filament_motion(int num, _filament_motion_state_set motion);
void loop()
{
    for (int i = 0; i < 4; i++)
    {
        set_filament_online(i, true);

    }
    set_filament_motion(0, need_send_out);
    while (1)
    {
        delay(100);
        Motion_control_run(0);
        RGB_update();
    }
}*/