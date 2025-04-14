#include "BambuBus.h"
#include <rs485_bsp.h>
#include "CRC16.h"
#include "CRC8.h"

#define FILAMENT_CONFIG_SAVE_ADDR   ((uint32_t)0x0800F000)
#define BAMBU_BUS_REV_BUF_LEN       (1000)

static uint32_t s_bambu_bus_receive_package_len = 0;
static uint8_t s_receive_dump[BAMBU_BUS_REV_BUF_LEN];
static uint8_t s_bambu_bus_rev_buf[BAMBU_BUS_REV_BUF_LEN];
static CRC8 s_crc8_rx_check;
static CRC16 s_crc16_check;
static CRC8 s_crc8_tx_check;

bambu_bus_device_type_t s_bambu_bus_device_type = BambuBus_none;

typedef struct _filament_info_
{
    // AMS statu
    char ID[8] = "GFG00";
    uint8_t color_R = 0xFF;
    uint8_t color_G = 0xFF;
    uint8_t color_B = 0xFF;
    uint8_t color_A = 0xFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";

    float meters = 0;
    uint64_t meters_virtual_count = 0;
    filament_state_t statu = online;
    // printer_set
    filament_motion_set_t motion_set = idle;
    uint16_t pressure = 0xFFFF;
}filament_info_t;

struct alignas(4) flash_save_struct
{
    filament_info_t filament[4][4];
    int BambuBus_now_filament_num = 0;
    uint32_t version = BAMBU_BUS_VER;
    uint32_t check = 0x40614061;
} data_save;

bool bambu_bus_load_config(void)
{
    flash_save_struct *ptr = (flash_save_struct *)(FILAMENT_CONFIG_SAVE_ADDR);
    if ((ptr->check == 0x40614061) && (ptr->version == BAMBU_BUS_VER))
    {
        memcpy(&data_save, ptr, sizeof(data_save));
        return true;
    }
    return false;
}

bool s_bambu_bus_config_updated = false;
void bambu_bus_save_config_later(void)
{
    s_bambu_bus_config_updated = true;
}

void bambu_bus_save_config_now(void)
{
    Flash_saves(&data_save, sizeof(data_save), FILAMENT_CONFIG_SAVE_ADDR);
}

int bambu_bus_get_filament_num(void)
{
    return data_save.BambuBus_now_filament_num;
}

bambu_bus_device_type_t bambu_bus_get_device_type()
{
    return s_bambu_bus_device_type;
}

/* MC模块调用此接口更新耗材使用长度 */
void bambu_bus_add_filament_meters(int num, float meters)
{
    if (num < 16)
    {
        int AMS = num / 4, filament = num % 4;
        if ((data_save.filament[AMS][filament].motion_set == on_use)||(data_save.filament[AMS][filament].motion_set == need_pull_back))
            data_save.filament[AMS][filament].meters += meters;
    }
}

/* 获取耗材的使用长度，此函数目前未使用 */
float bambu_bus_get_filament_meters(int num)
{
    if (num < 16)
        return data_save.filament[num / 4][num % 4].meters;
    else
        return 0;
}

/* 设置耗材通道的使用情况 */
void bambu_bus_set_filament_online_state(int num, bool if_online)
{
    if (num < 16)
    {
        if (if_online)
        {
            data_save.filament[num / 4][num % 4].statu = online;
        }
        else
        {
            data_save.filament[num / 4][num % 4].statu = offline;
        }
    }
}

bool bambu_bus_filament_is_online(int num)
{
    bool is_online = false;

    if (num < 16)
    {
        if (data_save.filament[num / 4][num % 4].statu == offline)
        {
            is_online = false;
        }
        else
        {
            is_online = true;
        }
    }
    return is_online;
}

/* 设置耗材通道的状态，此函数由MC模块调用。 */
void bambu_bus_set_filament_motion_state(int num, filament_motion_set_t motion)
{
    if (num < 16)
    {
        data_save.filament[num / 4][num % 4].motion_set = motion;
    }
}

/* 获取耗材通道运动状态设定，MC模块通过此函数读取总线请求。 */
filament_motion_set_t bambu_bus_get_filament_motion_set(int num)
{
    filament_motion_set_t motion_set = idle;
    if (num < 16)
    {
        motion_set = data_save.filament[num / 4][num % 4].motion_set;
    }
    return motion_set;
}

/* RS485总线接收处理。 */
static void inline bambu_bus_byte_receive_handler(uint8_t rev_byte)
{
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;

    if (_index == 0)
    {
        if (rev_byte == 0x3D)
        {
            s_bambu_bus_rev_buf[0] = 0x3D;
            s_crc8_rx_check.restart();
            s_crc8_rx_check.add(0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    }
    else
    {
        s_bambu_bus_rev_buf[_index] = rev_byte;
        if (_index == 1)
        {
            if (rev_byte & 0x80)
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else
            {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index)
        {
            length = rev_byte;
        }
        if (_index < data_CRC8_index)
        {
            s_crc8_rx_check.add(rev_byte);
        }
        else if (_index == data_CRC8_index)
        {
            if (rev_byte != s_crc8_rx_check.calc())
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length)
        {
            _index = 0;
            memcpy(s_receive_dump, s_bambu_bus_rev_buf, length);
            s_bambu_bus_receive_package_len = length;
        }
        if (_index >= 999)
        {
            _index = 0;
        }
    }
}

void bambu_bus_init(void)
{
    bool _init_ready = bambu_bus_load_config();
    s_crc8_tx_check.reset(0x39, 0x66, 0x00, false, false);
    s_crc8_rx_check.reset(0x39, 0x66, 0x00, false, false);
    s_crc16_check.reset(0x1021, 0x913D, 0, false, false);

    if (!_init_ready)
    {
        data_save.filament[0][0].color_R = 0xFF;
        data_save.filament[0][0].color_G = 0x00;
        data_save.filament[0][0].color_B = 0x00;
        data_save.filament[0][1].color_R = 0x00;
        data_save.filament[0][1].color_G = 0xFF;
        data_save.filament[0][1].color_B = 0x00;
        data_save.filament[0][2].color_R = 0x00;
        data_save.filament[0][2].color_G = 0x00;
        data_save.filament[0][2].color_B = 0xFF;
        data_save.filament[0][3].color_R = 0x88;
        data_save.filament[0][3].color_G = 0x88;
        data_save.filament[0][3].color_B = 0x88;

        data_save.filament[1][0].color_R = 0xC0;
        data_save.filament[1][0].color_G = 0x20;
        data_save.filament[1][0].color_B = 0x20;
        data_save.filament[1][1].color_R = 0x20;
        data_save.filament[1][1].color_G = 0xC0;
        data_save.filament[1][1].color_B = 0x20;
        data_save.filament[1][2].color_R = 0x20;
        data_save.filament[1][2].color_G = 0x20;
        data_save.filament[1][2].color_B = 0xC0;
        data_save.filament[1][3].color_R = 0x60;
        data_save.filament[1][3].color_G = 0x60;
        data_save.filament[1][3].color_B = 0x60;

        data_save.filament[2][0].color_R = 0x80;
        data_save.filament[2][0].color_G = 0x40;
        data_save.filament[2][0].color_B = 0x40;
        data_save.filament[2][1].color_R = 0x40;
        data_save.filament[2][1].color_G = 0x80;
        data_save.filament[2][1].color_B = 0x40;
        data_save.filament[2][2].color_R = 0x40;
        data_save.filament[2][2].color_G = 0x40;
        data_save.filament[2][2].color_B = 0x80;
        data_save.filament[2][3].color_R = 0x40;
        data_save.filament[2][3].color_G = 0x40;
        data_save.filament[2][3].color_B = 0x40;

        data_save.filament[3][0].color_R = 0x40;
        data_save.filament[3][0].color_G = 0x20;
        data_save.filament[3][0].color_B = 0x20;
        data_save.filament[3][1].color_R = 0x20;
        data_save.filament[3][1].color_G = 0x40;
        data_save.filament[3][1].color_B = 0x20;
        data_save.filament[3][2].color_R = 0x20;
        data_save.filament[3][2].color_G = 0x20;
        data_save.filament[3][2].color_B = 0x40;
        data_save.filament[3][3].color_R = 0x20;
        data_save.filament[3][3].color_G = 0x20;
        data_save.filament[3][3].color_B = 0x20;
    }
    for (auto &i : data_save.filament)
    {
        for (auto &j : i)
        {
#ifdef _Bambubus_DEBUG_mode_
            j.statu = online;
#else
            j.statu = offline;
#endif // DEBUG

            j.motion_set = idle;
            j.meters = 0;
        }
    }
    rs_485_bsp_register_byte_rev_callback(bambu_bus_byte_receive_handler);
    bambu_bus_init_usart();
}

bool package_check_crc16(uint8_t *data, int data_length)
{
    s_crc16_check.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        s_crc16_check.add(data[i]);
    }
    uint16_t num = s_crc16_check.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

bool need_debug = false;
void package_send_with_crc(uint8_t *data, int data_length)
{
    s_crc8_tx_check.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++)
        {
            s_crc8_tx_check.add(data[i]);
        }
        data[3] = s_crc8_tx_check.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++)
        {
            s_crc8_tx_check.add(data[i]);
        }
        data[6] = s_crc8_tx_check.calc();
    }
    s_crc16_check.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        s_crc16_check.add(data[i]);
    }
    uint16_t num = s_crc16_check.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    bambu_bus_send_data(data, data_length);
    if (need_debug)
    {
        RAW_LOG(data, data_length);
        need_debug = false;
    }
}

uint8_t packge_send_buf[1000];

#pragma pack(push, 1) // 将结构体按1字节对齐
struct long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas;
    uint16_t data_length;
};
#pragma pack(pop) // 恢复默认对齐

void Bambubus_long_package_send(long_packge_data *data)
{
    packge_send_buf[0] = 0x3D;
    packge_send_buf[1] = 0x00;
    data->package_length = data->data_length + 15;
    memcpy(packge_send_buf + 2, data, 11);
    memcpy(packge_send_buf + 13, data->datas, data->data_length);
    package_send_with_crc(packge_send_buf, data->data_length + 15);
}

void Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // +2byte CRC16
}

long_packge_data printer_data_long;
bambu_bus_package_type_t get_packge_type(uint8_t *buf, int length)
{
    if (package_check_crc16(buf, length) == false)
    {
        return BambuBus_package_NONE;
    }
    if (buf[1] == 0xC5)
    {

        switch (buf[4])
        {
        case 0x03:
            return BambuBus_package_filament_motion_short;
        case 0x04:
            return BambuBus_package_filament_motion_long;
        case 0x05:
            return BambuBus_package_online_detect;
        case 0x06:
            return BambuBus_package_REQx6;
        case 0x07:
            return BambuBus_package_NFC_detect;
        case 0x08:
            return BambuBus_package_set_filament;
        case 0x20:
            return BambuBus_package_heartbeat;
        default:
            return BambuBus_package_ETC;
        }
    }
    else if (buf[1] == 0x05)
    {
        Bambubus_long_package_analysis(buf, length, &printer_data_long);
        if (printer_data_long.target_address == BambuBus_AMS)
        {
            s_bambu_bus_device_type = BambuBus_AMS;
        }
        else if (printer_data_long.target_address == BambuBus_AMS_lite)
        {
            s_bambu_bus_device_type = BambuBus_AMS_lite;
        }

        switch (printer_data_long.type)
        {
        case 0x21A:
            return BambuBus_long_package_MC_online;
        case 0x211:
            return BambuBus_longe_package_filament;
        case 0x103:
        case 0x402:
            return BambuBus_long_package_version;
        default:
            return BambuBus_package_ETC;
        }
    }
    return BambuBus_package_NONE;
}
uint8_t package_num = 0;

uint8_t get_filament_left_char(uint8_t AMS_num)
{
    uint8_t data = 0;
    for (int i = 0; i < 4; i++)
    {
        if (data_save.filament[AMS_num][i].statu == online)
        {
            data |= (0x1 << i) << i; // 1<<(2*i)
            if (s_bambu_bus_device_type == BambuBus_AMS)
                if (data_save.filament[AMS_num][i].motion_set != idle)
                {
                    data |= (0x2 << i) << i; // 2<<(2*i)
                }
        }
    }
    return data;
}

void set_motion_res_datas(uint8_t *set_buf, uint8_t AMS_num, uint8_t read_num)
{
    float meters = 0;
    uint16_t pressure = 0xFFFF;
    uint8_t motion_flag = 0x00;
    if ((read_num != 0xFF) && (read_num < 4))
    {
        meters = data_save.filament[AMS_num][read_num].meters;
        pressure = data_save.filament[AMS_num][read_num].pressure;
        if ((data_save.filament[AMS_num][read_num].motion_set == idle)|| (data_save.filament[AMS_num][read_num].motion_set == need_pull_back)) // idle or pull back
        {
            motion_flag = 0x00;
        }
        else if ((data_save.filament[AMS_num][read_num].motion_set == need_send_out) ) // sending
        {
            motion_flag = 0x02;
        }
        else if ((data_save.filament[AMS_num][read_num].motion_set == on_use)) // on use
        {
            motion_flag = 0x04;
        }
    }
    set_buf[0] = AMS_num;
    set_buf[1] = 0x00;
    set_buf[2] = motion_flag;
    set_buf[3] = read_num; // filament number or maybe using number
    memcpy(set_buf + 4, &meters, sizeof(float));
    memcpy(set_buf + 8, &pressure, sizeof(uint16_t));
    set_buf[24] = get_filament_left_char(AMS_num);
}
bool set_motion(uint8_t AMS_num, uint8_t read_num, uint8_t statu_flags, uint8_t fliment_motion_flag)
{
    static uint64_t time_last = 0;
    uint64_t time_now = get_monotonic_timestamp64_ms();
    uint64_t time_used = time_now - time_last;
    time_last = time_now;
    if (s_bambu_bus_device_type == BambuBus_AMS) // AMS08
    {
        if (read_num < 4)
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00
            {
                uint8_t numx = AMS_num * 4 + read_num;
                if (data_save.BambuBus_now_filament_num != numx) // on change
                {
                    if (data_save.BambuBus_now_filament_num < 16)
                    {
                        data_save.filament[data_save.BambuBus_now_filament_num / 4][data_save.BambuBus_now_filament_num % 4].motion_set = idle;
                        data_save.filament[data_save.BambuBus_now_filament_num / 4][data_save.BambuBus_now_filament_num % 4].pressure = 0xFFFF;
                    }
                    data_save.BambuBus_now_filament_num = numx;
                }
                data_save.filament[AMS_num][read_num].motion_set = need_send_out;
                data_save.filament[AMS_num][read_num].pressure = 0x4700;
            }
            else if ((statu_flags == 0x09)) // 09 A5 / 09 3F
            {
                if (data_save.filament[AMS_num][read_num].motion_set == need_send_out)
                {
                    data_save.filament[AMS_num][read_num].motion_set = on_use;
                    data_save.filament[AMS_num][read_num].meters_virtual_count = 0;
                }
                else if (data_save.filament[AMS_num][read_num].meters_virtual_count < 10000) // 10s virtual data
                {
                    data_save.filament[AMS_num][read_num].meters += (float)time_used / 300000; // 3.333mm/s
                    data_save.filament[AMS_num][read_num].meters_virtual_count += time_used;
                }
                data_save.filament[AMS_num][read_num].pressure = 0x2B00;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x7F)) // 07 7F
            {
                data_save.filament[AMS_num][read_num].motion_set = on_use;
                data_save.filament[AMS_num][read_num].pressure = 0x2B00;
            }
        }
        else if ((read_num == 0xFF))
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00(FF)
            {
                filament_info_t *filament = &(data_save.filament[data_save.BambuBus_now_filament_num / 4][data_save.BambuBus_now_filament_num % 4]);
                if (data_save.BambuBus_now_filament_num < 16)
                {
                    if (filament->motion_set == on_use)
                        filament->motion_set = need_pull_back;
                    filament->pressure = 0x4700;
                }
            }
            else
            {
                for (auto i = 0; i < 4; i++)
                {
                    data_save.filament[AMS_num][i].motion_set = idle;
                    data_save.filament[AMS_num][i].pressure = 0xFFFF;
                }
            }
        }
    }
    else if (s_bambu_bus_device_type == BambuBus_AMS_lite) // AMS lite
    {
        if (read_num < 4)
        {
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x3F)) // 03 3F
            {
                data_save.filament[AMS_num][read_num].motion_set = need_pull_back;
            }
            else if ((statu_flags == 0x03) && (fliment_motion_flag == 0xBF)) // 03 BF
            {
                data_save.BambuBus_now_filament_num = AMS_num * 4 + read_num;
                if (data_save.filament[AMS_num][read_num].motion_set != need_send_out)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        data_save.filament[AMS_num][i].motion_set = idle;
                    }
                }
                data_save.filament[AMS_num][read_num].motion_set = need_send_out;
            }
            else if ((statu_flags == 0x07) && (fliment_motion_flag == 0x00)) // 07 00
            {
                data_save.BambuBus_now_filament_num = AMS_num * 4 + read_num;
                if (data_save.filament[AMS_num][read_num].motion_set == need_send_out)
                {
                    data_save.filament[AMS_num][read_num].motion_set = on_use;
                    data_save.filament[AMS_num][read_num].meters_virtual_count = 0;
                }
                else if (data_save.filament[AMS_num][read_num].meters_virtual_count < 10000) // 10s virtual data
                {
                    data_save.filament[AMS_num][read_num].meters += (float)time_used / 300000; // 3.333mm/s
                    data_save.filament[AMS_num][read_num].meters_virtual_count += time_used;
                }
                /*if (data_save.filament[AMS_num][read_num].motion_set == need_pull_back)
                    data_save.filament[AMS_num][read_num].motion_set = idle;*/
            }
        }
        else if ((read_num == 0xFF) && (statu_flags == 0x01))
        {
            filament_motion_set_t motion = data_save.filament[data_save.BambuBus_now_filament_num / 4][data_save.BambuBus_now_filament_num % 4].motion_set;
            if (motion != on_use)
                for (int i = 0; i < 4; i++)
                {
                    data_save.filament[AMS_num][i].motion_set = idle;
                }
        }
    }
    else if (s_bambu_bus_device_type == BambuBus_none) // none
    {
        /*if ((read_num != 0xFF) && (read_num < 4))
        {
            if ((statu_flags == 0x07) && (fliment_motion_flag == 0x00)) // 07 00
            {
                data_save.BambuBus_now_filament_num = AMS_num * 4 + read_num;
                data_save.filament[AMS_num][read_num].motion_set = on_use;
            }
        }*/
    }
    else
        return false;
    return true;
}
// 3D E0 3C 12 04 00 00 00 00 09 09 09 00 00 00 00 00 00 00
// 02 00 E9 3F 14 BF 00 00 76 03 6A 03 6D 00 E5 FB 99 14 2E 19 6A 03 41 F4 C3 BE E8 01 01 01 01 00 00 00 00 64 64 64 64 0A 27
// 3D E0 2C C9 03 00 00
// 04 01 79 30 61 BE 00 00 03 00 44 00 12 00 FF FF FF FF 00 00 44 00 54 C1 F4 EE E7 01 01 01 01 00 00 00 00 FA 35
#define C_test 0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x80, 0xBF, \
               0x00, 0x00, 0x00, 0x00, \
               0x36, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x27, 0x00, \
               0x55,                   \
               0xFF, 0xFF, 0xFF, 0xFF, \
               0xFF, 0xFF, 0xFF, 0xFF,
/*#define C_test 0x00, 0x00, 0x02, 0x02, \
               0x00, 0x00, 0x00, 0x00, \
               0x00, 0x00, 0x00, 0xC0, \
               0x36, 0x00, 0x00, 0x00, \
               0xFC, 0xFF, 0xFC, 0xFF, \
               0x00, 0x00, 0x27, 0x00, \
               0x55,                   \
               0xC1, 0xC3, 0xEC, 0xBC, \
               0x01, 0x01, 0x01, 0x01,
00 00 02 02 EB 8F CA 3F 49 48 E7 1C 97 00 E7 1B F3 FF F2 FF 00 00 90 00 75 F8 EE FC F0 B6 B8 F8 B0 00 00 00 00 FF FF FF FF*/
/*
#define C_test 0x00, 0x00, 0x02, 0x01, \
                0xF8, 0x65, 0x30, 0xBF, \
                0x00, 0x00, 0x28, 0x03, \
                0x2A, 0x03, 0x6F, 0x00, \
                0xB6, 0x04, 0xFC, 0xEC, \
                0xDF, 0xE7, 0x44, 0x00, \
                0x04, \
                0xC3, 0xF2, 0xBF, 0xBC, \
                0x01, 0x01, 0x01, 0x01,*/
uint8_t Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0x90, 0xE4};
void send_for_motion_short(uint8_t *buf, int length)
{
    Cxx_res[1] = 0xC0 | (package_num << 3);
    uint8_t AMS_num = buf[5];
    uint8_t statu_flags = buf[6];
    uint8_t read_num = buf[7];
    uint8_t fliment_motion_flag = buf[8];

    if (!set_motion(AMS_num, read_num, statu_flags, fliment_motion_flag))
        return;

    set_motion_res_datas(Cxx_res + 5, AMS_num, read_num);
    package_send_with_crc(Cxx_res, sizeof(Cxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}
/*
0x00, 0x00, 0x00, 0xFF, // 0x0C...
0x00, 0x00, 0x80, 0xBF, // distance
0x00, 0x00, 0x00, 0xC0,
0x00, 0xC0, 0x5D, 0xFF,
0xFE, 0xFF, 0xFE, 0xFF, // 0xFE, 0xFF, 0xFE, 0xFF,
0x00, 0x44, 0x00, 0x00,
0x10,
0xC1, 0xC3, 0xEC, 0xBC,
0x01, 0x01, 0x01, 0x01,
*/
uint8_t Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, //[5]AMS num
                           0x01,
                           0x01,
                           1,                      // humidity wet
                           0x04, 0x04, 0x04, 0xFF, // flags
                           0x00, 0x00, 0x00, 0x00,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x90, 0xE4};
/*uint8_t Dxx_res2[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                            0x00, 0x75, 0x01, 0x11,
                            0x0C, 0x04, 0x04, 0x03,
                            0x08, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x03, 0x03,
                            0x5F, 0x6E, 0xD7, 0xBE,
                            0x00, 0x00, 0x03, 0x00,
                            0x44, 0x00, 0x01, 0x00,
                            0xFE, 0xFF, 0xFE, 0xFF,
                            0x00, 0x00, 0x00, 0x00,
                            0x50,
                            0xC1, 0xC3, 0xED, 0xE9,
                            0x01, 0x01, 0x01, 0x01,
                            0x00, 0x00, 0x00, 0x00,
                            0xFF, 0xFF, 0xFF, 0xFF,
                            0xEC, 0xF0};*/
bool need_res_for_06 = false;
uint8_t res_for_06_num = 0xFF;
int last_detect = 0;
uint8_t filament_flag_detected = 0;

void send_for_motion_long(uint8_t *buf, int length)
{
    uint8_t filament_flag_on = 0x00;
    uint8_t filament_flag_NFC = 0x00;
    uint8_t AMS_num = buf[5];
    uint8_t statu_flags = buf[6];
    uint8_t fliment_motion_flag = buf[7];
    uint8_t read_num = buf[9];

    for (auto i = 0; i < 4; i++)
    {
        // filament[i].meters;
        if (data_save.filament[AMS_num][i].statu == online)
        {
            filament_flag_on |= 1 << i;
        }
        else if (data_save.filament[AMS_num][i].statu == NFC_waiting)
        {
            filament_flag_on |= 1 << i;
            filament_flag_NFC |= 1 << i;
        }
    }
    if (!set_motion(AMS_num, read_num, statu_flags, fliment_motion_flag))
        return;
    /*if (need_res_for_06)
    {
        Dxx_res2[1] = 0xC0 | (package_num << 3);
        Dxx_res2[9] = filament_flag_on;
        Dxx_res2[10] = filament_flag_on - filament_flag_NFC;
        Dxx_res2[11] = filament_flag_on - filament_flag_NFC;
        Dxx_res[19] = motion_flag;
        Dxx_res[20] = Dxx_res2[12] = res_for_06_num;
        Dxx_res2[13] = filament_flag_NFC;
        Dxx_res2[41] = get_filament_left_char();
        package_send_with_crc(Dxx_res2, sizeof(Dxx_res2));
        need_res_for_06 = false;
    }
    else*/

    {
        Dxx_res[1] = 0xC0 | (package_num << 3);
        Dxx_res[5] = AMS_num;
        Dxx_res[9] = filament_flag_on;
        Dxx_res[10] = filament_flag_on - filament_flag_NFC;
        Dxx_res[11] = filament_flag_on - filament_flag_NFC;
        Dxx_res[12] = read_num;
        Dxx_res[13] = filament_flag_NFC;

        set_motion_res_datas(Dxx_res + 17, AMS_num, read_num);
    }
    if (last_detect != 0)
    {
        if (last_detect > 10)
        {
            Dxx_res[19] = 0x01;
        }
        else
        {
            Dxx_res[12] = filament_flag_detected;
            Dxx_res[19] = 0x01;
            Dxx_res[20] = filament_flag_detected;
        }
        last_detect--;
    }
    package_send_with_crc(Dxx_res, sizeof(Dxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}
uint8_t REQx6_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x06,
                             0x00, 0x00, 0x00, 0x00,
                             0x04, 0x04, 0x04, 0xFF, // flags
                             0x00, 0x00, 0x00, 0x00,
                             C_test 0x00, 0x00, 0x00, 0x00,
                             0x64, 0x64, 0x64, 0x64,
                             0x90, 0xE4};
void send_for_REQx6(uint8_t *buf, int length)
{
    /*
        uint8_t filament_flag_on = 0x00;
        uint8_t filament_flag_NFC = 0x00;
        for (auto i = 0; i < 4; i++)
        {
            if (data_save.filament[AMS_num][i].statu == online)
            {
                filament_flag_on |= 1 << i;
            }
            else if (data_save.filament[AMS_num][i].statu == NFC_waiting)
            {
                filament_flag_on |= 1 << i;
                filament_flag_NFC |= 1 << i;
            }
        }
        REQx6_res[1] = 0xC0 | (package_num << 3);
        res_for_06_num = buf[7];
        REQx6_res[9] = filament_flag_on;
        REQx6_res[10] = filament_flag_on - filament_flag_NFC;
        REQx6_res[11] = filament_flag_on - filament_flag_NFC;
        Dxx_res2[12] = res_for_06_num;
        Dxx_res2[12] = res_for_06_num;
        package_send_with_crc(REQx6_res, sizeof(REQx6_res));
        need_res_for_06 = true;
        if (package_num < 7)
            package_num++;
        else
            package_num = 0;*/
}

void NFC_detect_run()
{
    /*uint64_t time = GetTick();
    return;
    if (time > last_detect + 3000)
    {
        filament_flag_detected = 0;
    }*/
}
uint8_t online_detect_num2[] = {0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, // 序列号？(额外包含之前一位)
                                0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t online_detect_num[] = {0x90, 0x31, 0x33, 0x34, 0x36, 0x35, 0x02, 0x00, 0x37, 0x39, 0x33, 0x38, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t F01_res[] = {
    0x3D, 0xC0, 0x1D, 0xB4, 0x05, 0x01, 0x00,
    0x16,
    0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00, 0x33, 0xF0};
void send_for_online_detect(uint8_t *buf, int length)
{
    /*
    uint8_t F00_res[4 * sizeof(F01_res)];
    if ((buf[5] == 0x00))
    {
        for (auto i = 0; i < 4; i++)
        {
            memcpy(F00_res + i * sizeof(F01_res), F01_res, sizeof(F01_res));
            F00_res[i * sizeof(F01_res) + 5] = 0;
            F00_res[i * sizeof(F01_res) + 6] = i;
            F00_res[i * sizeof(F01_res) + 7] = i;
        }
        package_send_with_crc(F00_res, sizeof(F00_res));
    }

    if ((buf[5] == 0x01) && (buf[6] < 4))
    {
        memcpy(F01_res + 4, buf + 4, 3);
        package_send_with_crc(F01_res, sizeof(F01_res));
    }*/
    uint8_t F00_res[sizeof(F01_res)];
    if ((buf[5] == 0x00))
    {
        memcpy(F00_res, F01_res, sizeof(F01_res));
        F00_res[5] = 0;
        F00_res[6] = 0;
        F00_res[7] = 0;

        package_send_with_crc(F00_res, sizeof(F00_res));
    }

    if ((buf[5] == 0x01) && (buf[6] == 0))
    {
        memcpy(F01_res + 4, buf + 4, 3);
        package_send_with_crc(F01_res, sizeof(F01_res));
    }
}
// 3D C5 0D F1 07 00 00 00 00 00 00 CE EC
// 3D C0 0D 6F 07 00 00 00 00 00 00 9A 70

uint8_t NFC_detect_res[] = {0x3D, 0xC0, 0x0D, 0x6F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xE8};
void send_for_NFC_detect(uint8_t *buf, int length)
{
    last_detect = 20;
    filament_flag_detected = 1 << buf[6];
    NFC_detect_res[6] = buf[6];
    NFC_detect_res[7] = buf[7];
    package_send_with_crc(NFC_detect_res, sizeof(NFC_detect_res));
}

uint8_t long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void send_for_long_packge_MC_online(uint8_t *buf, int length)
{
    long_packge_data data;
    uint8_t AMS_num = printer_data_long.datas[0];
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    if (printer_data_long.target_address == 0x0700)
    {
    }
    else if (printer_data_long.target_address == 0x1200)
    {
    }
    /*else if(printer_data_long.target_address==0x0F00)
    {

    }*/
    else
    {
        return;
    }

    data.datas = long_packge_MC_online;
    data.datas[0] = AMS_num;
    data.data_length = sizeof(long_packge_MC_online);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
uint8_t long_packge_filament[] =
    {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00,
        0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0xDD, 0xB1, 0xD4, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x18, 0x01, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
void send_for_long_packge_filament(uint8_t *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];
    long_packge_filament[0] = AMS_num;
    long_packge_filament[1] = filament_num;
    memcpy(long_packge_filament + 19, data_save.filament[AMS_num][filament_num].ID, sizeof(data_save.filament[AMS_num][filament_num].ID));
    memcpy(long_packge_filament + 27, data_save.filament[AMS_num][filament_num].name, sizeof(data_save.filament[AMS_num][filament_num].name));
    long_packge_filament[59] = data_save.filament[AMS_num][filament_num].color_R;
    long_packge_filament[60] = data_save.filament[AMS_num][filament_num].color_G;
    long_packge_filament[61] = data_save.filament[AMS_num][filament_num].color_B;
    long_packge_filament[62] = data_save.filament[AMS_num][filament_num].color_A;
    memcpy(long_packge_filament + 79, &data_save.filament[AMS_num][filament_num].temperature_max, 2);
    memcpy(long_packge_filament + 81, &data_save.filament[AMS_num][filament_num].temperature_min, 2);

    data.datas = long_packge_filament;
    data.data_length = sizeof(long_packge_filament);

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
uint8_t serial_number[] = {"STUDY0ONLY"};
uint8_t long_packge_version_serial_number[] = {9, // length
                                                     'S', 'T', 'U', 'D', 'Y', 'O', 'N', 'L', 'Y', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // serial_number#2
                                                     0x30, 0x30, 0x30, 0x30,
                                                     0xFF, 0xFF, 0xFF, 0xFF,
                                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

uint8_t long_packge_version_version_and_name_AMS_lite[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                                 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_version(uint8_t *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t *long_packge_version_version_and_name;

    if (printer_data_long.target_address == BambuBus_AMS)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS08;
    }
    else if (printer_data_long.target_address == BambuBus_AMS_lite)
    {
        long_packge_version_version_and_name = long_packge_version_version_and_name_AMS_lite;
    }
    else
    {
        return;
    }

    switch (printer_data_long.type)
    {
    case 0x402:

        AMS_num = printer_data_long.datas[33];
        long_packge_version_serial_number[0] = sizeof(serial_number);
        memcpy(long_packge_version_serial_number + 1, serial_number, sizeof(serial_number));
        data.datas = long_packge_version_serial_number;
        data.data_length = sizeof(long_packge_version_serial_number);

        data.datas[65] = AMS_num;
        break;
    case 0x103:

        AMS_num = printer_data_long.datas[0];
        data.datas = long_packge_version_version_and_name;
        data.data_length = sizeof(long_packge_version_version_and_name_AMS08);
        data.datas[20] = AMS_num;
        break;
    default:
        return;
    }

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
uint8_t s = 0x01;

uint8_t Set_filament_res[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
void send_for_set_filament(uint8_t *buf, int length)
{
    uint8_t read_num = buf[5];
    uint8_t AMS_num = read_num & 0xF0;
    read_num = read_num & 0x0F;
    memcpy(data_save.filament[AMS_num][read_num].ID, buf + 7, sizeof(data_save.filament[AMS_num][read_num].ID));

    data_save.filament[AMS_num][read_num].color_R = buf[15];
    data_save.filament[AMS_num][read_num].color_G = buf[16];
    data_save.filament[AMS_num][read_num].color_B = buf[17];
    data_save.filament[AMS_num][read_num].color_A = buf[18];

    memcpy(&data_save.filament[AMS_num][read_num].temperature_min, buf + 19, 2);
    memcpy(&data_save.filament[AMS_num][read_num].temperature_max, buf + 21, 2);
    memcpy(data_save.filament[AMS_num][read_num].name, buf + 23, sizeof(data_save.filament[AMS_num][read_num].name));
    package_send_with_crc(Set_filament_res, sizeof(Set_filament_res));
    bambu_bus_save_config_later();
}

bambu_bus_package_type_t bambu_bus_ticks_handler(void)
{
    bambu_bus_package_type_t stu = BambuBus_package_NONE;
    static uint64_t time_set = 0;
    static uint64_t time_motion = 0;

    uint64_t timex = get_monotonic_timestamp64_ms();

    /*for (auto i : data_save.filament)
    {
        i->motion_set = idle;
    }*/

    if (s_bambu_bus_receive_package_len)
    {
        int data_length = s_bambu_bus_receive_package_len;
        s_bambu_bus_receive_package_len = 0;
        need_debug = false;
        delay(1);
        stu = get_packge_type(s_receive_dump, data_length); // have_data
        switch (stu)
        {
        case BambuBus_package_heartbeat:
            time_set = timex + 1000;
            break;
        case BambuBus_package_filament_motion_short:
            send_for_motion_short(s_receive_dump, data_length);
            break;
        case BambuBus_package_filament_motion_long:
            RAW_LOG(s_receive_dump, data_length);
            send_for_motion_long(s_receive_dump, data_length);
            time_motion = timex + 1000;
            break;
        case BambuBus_package_online_detect:

            send_for_online_detect(s_receive_dump, data_length);
            break;
        case BambuBus_package_REQx6:
            // send_for_REQx6(buf_X, data_length);
            break;
        case BambuBus_long_package_MC_online:
            send_for_long_packge_MC_online(s_receive_dump, data_length);
            break;
        case BambuBus_longe_package_filament:
            send_for_long_packge_filament(s_receive_dump, data_length);
            break;
        case BambuBus_long_package_version:
            send_for_long_packge_version(s_receive_dump, data_length);
            break;
        case BambuBus_package_NFC_detect:
            // send_for_NFC_detect(buf_X, data_length);
            break;
        case BambuBus_package_set_filament:
            send_for_set_filament(s_receive_dump, data_length);
            break;
        default:
            break;
        }
    }
    if (timex > time_set)
    {
        stu = BambuBus_package_ERROR; // offline
    }
    if (timex > time_motion)
    {
        // set_filament_motion(bambu_bus_get_filament_num(),idle);
        /*for(auto i:data_save.filament)
        {
            i->motion_set=idle;
        }*/
    }
    if (s_bambu_bus_config_updated)
    {
        bambu_bus_save_config_now();
        time_set = get_monotonic_timestamp64_ms() + 1000;
        s_bambu_bus_config_updated = false;
    }
    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);

    // NFC_detect_run();
    return stu;
}
