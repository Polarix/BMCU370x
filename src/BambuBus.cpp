#include "BambuBus.h"
#include "Flash_saves.h"
#include <bsp/rs485_bsp.h>
#include <bsp/on_chip_flash.h>
#include "Debug_log.h"
#include "CRC16.h"
#include "algorithm/crc_bambu_bus.h"

CRC16 crc_16;
static crc8_t s_tx_crc8_cala;
static crc8_t s_rx_crc8_cala;

uint8_t s_usart_rev_buf[1000];
static uint32_t s_bambu_bus_rev_length = 0;
uint16_t BambuBus_address = 0;
uint8_t AMS_num = 1;

typedef struct _st_filament_channel_
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
    _filament_status statu = online;
    // printer_set
    _filament_motion_state_set motion_set = idle;
    uint16_t pressure = 0xFFFF;
}bambu_bus_filament_channel_t;

#define use_flash_addr ((uint32_t)0x0800F000)

struct alignas(4) flash_save_struct
{
    bambu_bus_filament_channel_t filament[4][4]; /* 下标含义为[AMS设备索引][设备上的料盘索引] */
    int actived_channel = 0;
    uint32_t version = BAMBU_BUS_VER;
    uint32_t check = 0x40614061;
} data_save;

bool bambu_bus_load_storage_data(void)
{
    flash_save_struct *ptr = (flash_save_struct *)(use_flash_addr);
    if ((ptr->check == 0x40614061) && (ptr->version == BAMBU_BUS_VER))
    {
        memcpy(&data_save, ptr, sizeof(data_save));
        return true;
    }
    return false;
}

static bool s_bambu_bus_need_to_save = false;
void bambu_bus_need_to_save(void)
{
    s_bambu_bus_need_to_save = true;
}

/* 保存耗材信息至Flash */
void bambu_bus_save_storage_data(void)
{
    on_chip_flash_save(&data_save, sizeof(data_save), use_flash_addr);
}

int bambu_bus_get_actived_filament(void)
{
    return data_save.actived_channel;
}
uint16_t get_now_BambuBus_device_type()
{
    return BambuBus_address;
}

/* 复位料丝通道计数 */
void reset_filament_meters(int num)
{
    /* 最多四台AMS、每个AMS最多四通道，共计十六个料丝通道。 */
    if (num < 16)
    {
        data_save.filament[num / 4][num % 4].meters = 0;
    }
}

/* 设置料丝增量 */
void add_filament_meters(int num, float meters)
{
    /* 最多四台AMS、每个AMS最多四通道，共计十六个料丝通道。 */
    if (num < 16)
    {
        int AMS = num / 4, filament = num % 4;
        if ((data_save.filament[AMS][filament].motion_set == on_use)||(data_save.filament[AMS][filament].motion_set == need_pull_back))
        {
            data_save.filament[AMS][filament].meters += meters;
        }
    }
}

/* 获取当前通道料丝使用量 */
float get_filament_meters(int num)
{
    if (num < 16)
        return data_save.filament[num / 4][num % 4].meters;
    else
        return 0;
}

void set_filament_online(int num, bool if_online)
{
    if(num < 16)
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

bool get_filament_online(int num)
{
    bool result = false;

    if (num < 16)
    {
        if (data_save.filament[num / 4][num % 4].statu != offline)
        {
            result = true;
        }
    }
    return result;
}

/* 获取耗材丝状态 */
/* 此函数被MC模块调用，用于更新各通道的运动状态，用于通信反馈 */
void bambu_bus_set_filament_motion(int num, _filament_motion_state_set motion)
{
    if (num < 16)
        data_save.filament[num / 4][num % 4].motion_set = motion;
}

/* 获取耗材丝状态 */
/* 此函数被MC模块调用，用于根据各通道的设定值更新电机的运动状态 */
_filament_motion_state_set bambu_bus_get_filament_motion(int num)
{
    if (num < 16)
        return data_save.filament[num / 4][num % 4].motion_set;
    else
        return idle;
}

/* 当前是否在打印中 */
bool bambu_bus_is_on_printing(void)
{
    bool on_print = false;
    /* 逐一判断各个通道的状态，只要不是IDLE就是在使用中，使用中就意味着正在打印。 */
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; i < 4; j++)
        {
            if (data_save.filament[i][j].motion_set != idle)
            {
                on_print = true;
            }
        }
    }
    return on_print;
}

uint8_t s_usart_rev_dump[1000]; /* 串口接收完成后的内容会被转储到这里 */

/* 串口中断服务函数 */
static void inline banbu_bus_byte_receive_handler(uint8_t rev_byte)
{
    static int _index = 0;
    static int length = 500;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;
    unsigned char data = rev_byte;

    if (_index == 0)
    {
        /* 每个数据包都是以0x3D开头 */
        if (data == 0x3D)
        {
            s_usart_rev_buf[0] = 0x3D;
            /* 新的数据包传输开始，重置CRC计算。 */
            bambu_bus_crc8_init(&s_rx_crc8_cala);
            bambu_bus_crc8_step(&s_rx_crc8_cala, 0x3D);
            data_length_index = 4;
            length = data_CRC8_index = 6;
            _index = 1;
        }
        return;
    }
    else
    {
        s_usart_rev_buf[_index] = data;
        if (_index == 1)
        {
            /* 第二个字节 */
            if (data & 0x80)
            {
                data_length_index = 2;  /* 索引第2字节为数据包长度 */
                data_CRC8_index = 3;    /* 索引第3字节为CRC8校验值的位置 */
            }
            else
            {
                data_length_index = 4;  /* 索引第4字节为数据包长度 */
                data_CRC8_index = 6;    /* 索引第6字节为CRC8校验值的位置 */
            }
        }
        if (_index == data_length_index)
        {
            /* 数据包长度 */
            length = data;
        }
        if (_index < data_CRC8_index)
        {
            /* 将CRC8校验值之前的所有数据都参与CRC8计算 */
            bambu_bus_crc8_step(&s_rx_crc8_cala, data);
        }
        else if (_index == data_CRC8_index)
        {
            /* 计算CRC8并比较，如果错误则忽略此次传输，重置索引等待重新开始。 */
            if (data != bambu_bus_crc8_finialize(&s_rx_crc8_cala))
            {
                _index = 0;
                return;
            }
        }
        ++_index;
        if (_index >= length)
        {
            _index = 0;
            /* 已接收到一个完整的数据包。*/
            
            memcpy(s_usart_rev_dump, s_usart_rev_buf, length);
            s_bambu_bus_rev_length = length;
        }
        if (_index >= 999)
        {
            /* 无效的数据包长度 */
            /* 重置索引，等待下一次传输。 */
        _index = 0;
        }
    }
}

#include <stdio.h>

/* 通过DMA发送Respose给打印机。 */
void bambu_bus_bsp_uart_write(const void* data, uint16_t length)
{
    rs_485_bsp_write(data, length);
}

#if 0
extern "C" void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        banbu_bus_byte_receive_handler(USART_ReceiveData(USART1));
    }
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        /* 发送完毕，清空RS485控制器的发送线。 */
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        GPIOA->BCR = GPIO_Pin_12;
    }
}
#endif

void bambu_bus_init(void)
{
    flash_storage_init();

    bool _init_ready = bambu_bus_load_storage_data();
    //crc_8.reset(0x39, 0x66, 0, false, false);
    bambu_bus_crc8_init(&s_tx_crc8_cala);
    crc_16.reset(0x1021, 0x913D, 0, false, false);

    /* 初次使用时，初始化BMCU的保存数据。 */
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
    /* 初始化各通道状态和送料长度计数。 */
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
    /* 注册字节接收处理函数，提前注册以防止漏接收字节。 */
    rs_485_bsp_register_byte_rev_callback(banbu_bus_byte_receive_handler);
    /* 初始化RS485总线。 */
    rs_485_bsp_init();
}

bool package_check_crc16(const uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    /* 最后两字节为CRC16的值，之前的是数据。 */
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}
bool need_debug = false;
/* 计算CRC值并发送响应数据 */
void bambu_bus_send_response(uint8_t *data, int data_length)
{
    /* 计算包头的CRC8校验值。 */
    bambu_bus_crc8_init(&s_tx_crc8_cala);
    if (data[1] & 0x80)
    {
        /* 短包应答的头部CRC8计算 */

        for (auto i = 0; i < 3; i++)
        {
            bambu_bus_crc8_step(&s_tx_crc8_cala, data[i]);
        }
        data[3] = bambu_bus_crc8_finialize(&s_tx_crc8_cala);
    }
    else
    {
        /* 长包应答的头部CRC8计算 */
        for (auto i = 0; i < 6; i++)
        {
            bambu_bus_crc8_step(&s_tx_crc8_cala, data[i]);
        }
        data[6] = bambu_bus_crc8_finialize(&s_tx_crc8_cala);
    }
    /* 计算整包的CRC16校验值。 */
    crc_16.restart();
    data_length -= 2; /* 忽略包尾的2字节CRC16校验值。 */
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[(data_length)] = num & 0xFF;
    data[(data_length + 1)] = num >> 8;
    data_length += 2;
    /* 通过USART发送响应数据。 */
    bambu_bus_bsp_uart_write(data, data_length);
    if (need_debug)
    {
        DEBUG_NUM_OUT(data, data_length);
        need_debug = false;
    }
}

uint8_t packge_send_buf[1000];

#pragma pack(push, 1) // 将结构体按1字节对齐
struct long_packge_data
{
    uint16_t package_number;    // 2
    uint16_t package_length;    // 2
    uint8_t crc8;               // 1
    uint16_t target_address;    // 2
    uint16_t source_address;    // 2
    uint16_t type;              // 2
    uint8_t *datas;             // 4
    uint16_t data_length;       // 2
};
#pragma pack(pop) // 恢复默认对齐

/* 发送长数据包应答数据 */
void Bambubus_long_package_send(long_packge_data *data)
{
    /* 包头固定值 */
    packge_send_buf[0] = 0x3D;
    packge_send_buf[1] = 0x00;
    data->package_length = data->data_length + 15; /* 2字节先导头+11字节长包头+2字节尾部CRC16 */
    memcpy(packge_send_buf + 2, data, 11);
    memcpy(packge_send_buf + 13, data->datas, data->data_length);
    /* 计算CRC值并发送数据 */
    bambu_bus_send_response(packge_send_buf, data->data_length + 15);
}

void Bambubus_long_package_analysis(uint8_t *buf, int data_length, long_packge_data *data)
{
    /* |01|2--12|13-14|15| */
    /* 忽略0/1字节，从第2-12字节是长数据包的内容，长度11字节。 */
    memcpy(data, buf + 2, 11);
    /* 13字节开始是包数据，长度未知 */
    data->datas = buf + 13;
    /* 再减去2字节的CRC16校验值，其余的是包数据体。 */
    data->data_length = data_length - (13+2); // +2byte CRC16
}

long_packge_data printer_data_long;
bambu_bus_package_type_t bambu_bus_get_package_type(uint8_t* buf, int length)
{
    /* 首先计算接收包的CRC16，判断数据有效性。 */
    if (package_check_crc16(buf, length) == false)
    {
        return BambuBus_package_NONE;
    }
    /* 字节1是C5 */
    if (buf[1] == 0xC5)
    {
        /* 根据字节4判断数据类型。 */
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
            return BambuBus_package_ETC; // 不需要应答处理
        }
    }
    /* 字节1是05，代表接手的是“长数据”，需要对数据进行二次解析。 */
    else if (buf[1] == 0x05)
    {
        Bambubus_long_package_analysis(buf, length, &printer_data_long);

        if (printer_data_long.target_address == BambuBus_AMS)
        {
            BambuBus_address = BambuBus_AMS;
        }
        else if (printer_data_long.target_address == BambuBus_AMS_lite)
        {
            BambuBus_address = BambuBus_AMS_lite;
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
    /* 未知包类型，丢弃。 */
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
            if (BambuBus_address == BambuBus_AMS)
            {
                if (data_save.filament[AMS_num][i].motion_set != idle)
                {
                    data |= (0x2 << i) << i; // 2<<(2*i)
                }
            }
        }
    }
    return data;
}

/* 设置运动控制(MC)响应数据 */
void set_motion_res_datas(unsigned char *set_buf, unsigned char AMS_num, unsigned char read_num)
{
    float meters = 0; /* 这个为什么是浮点型？ */
    uint16_t pressure = 0xFFFF;
    uint8_t motion_flag = 0x00;
    if ((read_num != 0xFF) && (read_num < 4))
    {
        /* 料丝长度 */
        meters = data_save.filament[AMS_num][read_num].meters;
        /* 耗材压力 */
        pressure = data_save.filament[AMS_num][read_num].pressure;
        /* 运动状态 */
        if ((data_save.filament[AMS_num][read_num].motion_set == idle)|| (data_save.filament[AMS_num][read_num].motion_set == need_pull_back)) // idle or pull back
        {
            /* 空闲或回抽 */
            motion_flag = 0x00;
        }
        else if ((data_save.filament[AMS_num][read_num].motion_set == need_send_out) ) // sending
        {
            /* 出料/续料 */
            motion_flag = 0x02;
        }
        else if ((data_save.filament[AMS_num][read_num].motion_set == on_use)) // on use
        {
            /* 使用中 */
            motion_flag = 0x04;
        }
    }
    set_buf[0] = AMS_num; /* 设备索引 */
    set_buf[1] = 0x00;
    set_buf[2] = motion_flag;   /* 运动状态 */
    set_buf[3] = read_num;  /* 通道号 */ // filament number or maybe using number
    memcpy(set_buf + 4, &meters, sizeof(float));
    memcpy(set_buf + 8, &pressure, sizeof(uint16_t));
    set_buf[24] = get_filament_left_char(AMS_num);
}

/* 更新挤出电机的状态 */
bool set_motion(unsigned char AMS_num, unsigned char read_num, unsigned char statu_flags, unsigned char fliment_motion_flag)
{
    static uint64_t time_last = 0;
    uint64_t time_now = get_time64();
    uint64_t time_used = time_now - time_last;
    time_last = time_now;
    if (BambuBus_address == BambuBus_AMS) // AMS08
    {
        if (read_num < 4)
        {
            /* [?] 0x03 0x00 为通道状态更新。 */
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00
            {
                uint8_t numx = AMS_num * 4 + read_num;
                /* 与当前活动通道作比较 */
                if (data_save.actived_channel != numx) // on change
                {
                    /* 如果激活了新的通道，则当前通道设置为IDLE状态 */
                    if (data_save.actived_channel < 16)
                    {
                        data_save.filament[data_save.actived_channel / 4][data_save.actived_channel % 4].motion_set = idle;
                        data_save.filament[data_save.actived_channel / 4][data_save.actived_channel % 4].pressure = 0xFFFF;
                    }
                    /* 记录新的活动通道 */
                    data_save.actived_channel = numx;
                }
                /* 设定新的活动通道状态为出料状态，压力值0x4700 */
                data_save.filament[AMS_num][read_num].motion_set = need_send_out;
                data_save.filament[AMS_num][read_num].pressure = 0x4700;
            }
            else if ((statu_flags == 0x09)) // 09 A5 / 09 3F
            {
                if (data_save.filament[AMS_num][read_num].motion_set == need_send_out)
                {
                    /* need_send_out状态：当前没有通道正在激活和使用，但是有一个通道被指定向挤出机中送料，即将被使用。 */
                    /* 如果通道正在出料(need_send_out)，则更新状态为使用中 */
                    /* 出料状态下首次收到这个指令时清空耗材长度计数(meters_virtual_count)并切换状态至使用中(on_use) */
                    data_save.filament[AMS_num][read_num].motion_set = on_use;
                    data_save.filament[AMS_num][read_num].meters_virtual_count = 0;
                }
                else if (data_save.filament[AMS_num][read_num].meters_virtual_count < 10000) // 10s virtual data
                {
                    /* 使用中状态下(on_use)状态下接收到这个指令要累加耗材计数。 */
                    /* meters累加值是时间？为什么meters是浮点但meters_virtual_count是整数？怀疑有歧义。 */
                    data_save.filament[AMS_num][read_num].meters += (float)time_used / 300000; // 3.333mm/s
                    data_save.filament[AMS_num][read_num].meters_virtual_count += time_used;
                }
                /* 更新通道压力值 */
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
            /* [?] 0x03 0x00 为通道状态更新。 */
            /* 通道索引为FF则代表无效通道，需要回抽耗材 */
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x00)) // 03 00(FF)
            {
                bambu_bus_filament_channel_t *filament = &(data_save.filament[data_save.actived_channel / 4][data_save.actived_channel % 4]);
                if (data_save.actived_channel < 16)
                {
                    /* 如果发现有使用中的通道，则更换新状态为回抽状态 */
                    if (filament->motion_set == on_use)
                    {
                        filament->motion_set = need_pull_back;
                    }
                    /* 更新压力值？ */
                    filament->pressure = 0x4700;
                }
            }
            /* 更新指定设备的料丝状态为IDLE */
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
    else if (BambuBus_address == BambuBus_AMS_lite) // AMS lite
    {
        if (read_num < 4)
        {            
            if ((statu_flags == 0x03) && (fliment_motion_flag == 0x3F)) // 03 3F
            {
                /* 0x03 0x3F 回抽料丝 */
                data_save.filament[AMS_num][read_num].motion_set = need_pull_back;
            }
            else if ((statu_flags == 0x03) && (fliment_motion_flag == 0xBF)) // 03 BF
            {
                /* 0x03 0xBF 送出料丝 */
                data_save.actived_channel = AMS_num * 4 + read_num;
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
                /* 0x07 0x00 开始送料 */
                data_save.actived_channel = AMS_num * 4 + read_num;
                if (data_save.filament[AMS_num][read_num].motion_set == need_send_out)
                {
                    /* 切换到使用状态 */
                    data_save.filament[AMS_num][read_num].motion_set = on_use;
                    data_save.filament[AMS_num][read_num].meters_virtual_count = 0;
                }
                else if (data_save.filament[AMS_num][read_num].meters_virtual_count < 10000) // 10s virtual data
                {
                    /* 应该是计算增量，具体没看懂啥意思。 */
                    data_save.filament[AMS_num][read_num].meters += (float)time_used / 300000; // 3.333mm/s
                    data_save.filament[AMS_num][read_num].meters_virtual_count += time_used;
                }
                /*if (data_save.filament[AMS_num][read_num].motion_set == need_pull_back)
                    data_save.filament[AMS_num][read_num].motion_set = idle;*/
            }
        }
        else if ((read_num == 0xFF) && (statu_flags == 0x01))
        {
            /* 通道索引为FF则代表无效通道 */
            _filament_motion_state_set motion = data_save.filament[data_save.actived_channel / 4][data_save.actived_channel % 4].motion_set;
            if (motion != on_use)
            {
                /* 停止通道动作，设置所有通道为空闲状态 */
                for (int i = 0; i < 4; i++)
                {
                    data_save.filament[AMS_num][i].motion_set = idle;
                }
            }
        }
    }
    else if (BambuBus_address == BambuBus_none) // none
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
    {
        return false;
    }
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
/*
#define C_test 0x00, 0x00, 0x02, 0x02, \
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
#define C_test  0x00, 0x00, 0x02, 0x01, \
                0xF8, 0x65, 0x30, 0xBF, \
                0x00, 0x00, 0x28, 0x03, \
                0x2A, 0x03, 0x6F, 0x00, \
                0xB6, 0x04, 0xFC, 0xEC, \
                0xDF, 0xE7, 0x44, 0x00, \
                0x04, \
                0xC3, 0xF2, 0xBF, 0xBC, \
                0x01, 0x01, 0x01, 0x01,*/
unsigned char Cxx_res[] = {0x3D, 0xE0, 0x2C, 0x1A, 0x03,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0x90, 0xE4};
void send_for_motion_short(unsigned char *buf, int length)
{
    /* 这个package_num是干啥用的？ */
    Cxx_res[1] = 0xC0 | (package_num << 3);
    unsigned char AMS_num = buf[5]; /* 设备索引 */
    unsigned char statu_flags = buf[6];
    unsigned char read_num = buf[7]; /* 通道索引 */
    unsigned char fliment_motion_flag = buf[8];

    if (!set_motion(AMS_num, read_num, statu_flags, fliment_motion_flag))
        return;

    set_motion_res_datas(Cxx_res + 5, AMS_num, read_num);

    bambu_bus_send_response(Cxx_res, sizeof(Cxx_res));
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
unsigned char Dxx_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
                           0x00, //[5]AMS num
                           0x01,
                           0x01,
                           1,                      // humidity wet
                           0x04, 0x04, 0x04, 0xFF, // flags
                           0x00, 0x00, 0x00, 0x00,
                           C_test 0x00, 0x00, 0x00, 0x00,
                           0xFF, 0xFF, 0xFF, 0xFF,
                           0x90, 0xE4};
/*unsigned char Dxx_res2[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x04,
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

void send_for_motion_long(unsigned char *buf, int length)
{
    unsigned char filament_flag_on = 0x00;
    unsigned char filament_flag_NFC = 0x00;
    unsigned char AMS_num = buf[5];
    unsigned char statu_flags = buf[6];
    unsigned char fliment_motion_flag = buf[7];
    unsigned char read_num = buf[9];

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
    bambu_bus_send_response(Dxx_res, sizeof(Dxx_res));
    if (package_num < 7)
        package_num++;
    else
        package_num = 0;
}
unsigned char REQx6_res[] = {0x3D, 0xE0, 0x3C, 0x1A, 0x06,
                             0x00, 0x00, 0x00, 0x00,
                             0x04, 0x04, 0x04, 0xFF, // flags
                             0x00, 0x00, 0x00, 0x00,
                             C_test 0x00, 0x00, 0x00, 0x00,
                             0x64, 0x64, 0x64, 0x64,
                             0x90, 0xE4};
void send_for_REQx6(unsigned char *buf, int length)
{
    /*
        unsigned char filament_flag_on = 0x00;
        unsigned char filament_flag_NFC = 0x00;
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

//===========================================================//
//= 设备上线检知响应                                         =//
//===========================================================//
/* 数据模板 */
static uint8_t online_detect_num2[] = {0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, // 序列号？(额外包含之前一位)
                                0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t online_detect_num[] = {0x90, 0x31, 0x33, 0x34, 0x36, 0x35, 0x02, 0x00, 0x37, 0x39, 0x33, 0x38, 0xFF, 0xFF, 0xFF, 0xFF};

static uint8_t s_f01_resp_template[] =
{
    /*  0 */0x3D, 0xC0, 
    /*  2 */0x1D, 0xB4, 0x05, 0x01, 0x00,
    /*  7 */0x16,
    /*  8 */0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00, 0x36, 0x39, 0x37, 0x33, 0xFF, 0xFF, 0xFF, 0xFF,
    /* 24 */0x00, 0x00, 0x00, 0x33, 0xF0
};
/* 设备上线检知响应函数 */
void send_for_online_detect(uint8_t* buf, int length)
{
#if 0
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
    }
#endif
    uint8_t F00_res[sizeof(s_f01_resp_template)];
    // const uint8_t* bptr = reinterpret_cast<const uint8_t*>(buf);
    if ((buf[5] == 0x00))
    {
        memcpy(F00_res, s_f01_resp_template, sizeof(s_f01_resp_template));
        F00_res[5] = 0;
        F00_res[6] = 0;
        F00_res[7] = 0;

        bambu_bus_send_response(F00_res, sizeof(F00_res));
    }

    if ((buf[5] == 0x01) && (buf[6] == 0))
    {
        memcpy(s_f01_resp_template + 4, buf + 4, 3);
        bambu_bus_send_response(s_f01_resp_template, sizeof(s_f01_resp_template));
    }
}
// 3D C5 0D F1 07 00 00 00 00 00 00 CE EC
// 3D C0 0D 6F 07 00 00 00 00 00 00 9A 70

//===========================================================//
//= NFC检知响应                                             =//
//===========================================================//
/* 数据模板 */
unsigned char NFC_detect_res[] = {0x3D, 0xC0, 0x0D, 0x6F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xE8};
/* NFC检知响应函数 */
void send_for_NFC_detect(unsigned char *buf, int length)
{
    last_detect = 20;
    filament_flag_detected = 1 << buf[6];
    NFC_detect_res[6] = buf[6];
    NFC_detect_res[7] = buf[7];
    bambu_bus_send_response(NFC_detect_res, sizeof(NFC_detect_res));
}

unsigned char long_packge_MC_online[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
/* 返回电机控制状态， 实际没做什么处理 */
void send_for_long_packge_MC_online(unsigned char *buf, int length)
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

//===========================================================//
//= 获取耗材信息响应                                         =//
//===========================================================//
/* 数据模板 */
unsigned char long_packge_filament[] =
    {
        /*  0 */0x00, /* 设备索引 */
        /*  1 */0x00, /* 耗材通道索引 */
        /*  2 */0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 未知用途 */
        /* 19 */0x47, 0x46, 0x42, 0x30, 0x30, 0x00, 0x00, 0x00, /* 耗材ID */
        /* 27 */0x41, 0x42, 0x53, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 耗材名 */
        /* 47 */0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 未知用途 */
        /* 59 */0xDD, 0xB1, 0xD4, 0xFF, /* 耗材颜色 */
        /* 63 */0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 未知用途 */
        /* 79 */0x18, 0x01, /* 最高打印温度 */
        /* 81 */0xF0, 0x00, /* 最低打印温度 */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
/* 获取耗材信息响应函数 */
void send_for_long_packge_filament(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);

    uint8_t AMS_num = printer_data_long.datas[0];
    uint8_t filament_num = printer_data_long.datas[1];
    /* 每次只获取一个通道 */
    long_packge_filament[0] = AMS_num;      /* 设备索引 */
    long_packge_filament[1] = filament_num; /* 耗材通道索引 */
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

    /* 发送长包应答数据 */
    Bambubus_long_package_send(&data);
}

//===========================================================//
//= 获取版本信息响应                                         =//
//===========================================================//
/* 数据模板 */
unsigned char serial_number[] = {"STUDY0ONLY"};
unsigned char long_packge_version_serial_number[] = {9, // length
                                                     'S', 'T', 'U', 'D', 'Y', 'O', 'N', 'L', 'Y', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // serial_number#2
                                                     0x30, 0x30, 0x30, 0x30,
                                                     0xFF, 0xFF, 0xFF, 0xFF,
                                                     0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBB, 0x44, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};

unsigned char long_packge_version_version_and_name_AMS_lite[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                                 0x41, 0x4D, 0x53, 0x5F, 0x46, 0x31, 0x30, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char long_packge_version_version_and_name_AMS08[] = {0x00, 0x00, 0x00, 0x00, // verison number
                                                              0x41, 0x4D, 0x53, 0x30, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void send_for_long_packge_version(unsigned char *buf, int length)
{
    long_packge_data data;
    Bambubus_long_package_analysis(buf, length, &printer_data_long);
    uint8_t AMS_num = printer_data_long.datas[0];
    unsigned char *long_packge_version_version_and_name;

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
            /* Unknown target, do nothing and exit. */
            return;
    }

    switch (printer_data_long.type)
    {
        case 0x402: /* 获取串号 */
        {
            AMS_num = printer_data_long.datas[33];
            long_packge_version_serial_number[0] = sizeof(serial_number);
            memcpy(long_packge_version_serial_number + 1, serial_number, sizeof(serial_number));
            data.datas = long_packge_version_serial_number;
            data.data_length = sizeof(long_packge_version_serial_number);

            data.datas[65] = AMS_num;
            break;
        }
        case 0x103:
        {
            AMS_num = printer_data_long.datas[0];
            data.datas = long_packge_version_version_and_name;
            data.data_length = sizeof(long_packge_version_version_and_name_AMS08);
            data.datas[20] = AMS_num;
            break;
        }
        default:
        {
            /* Unknown type, do nothing and exit. */
            return;
        }
    }

    data.package_number = printer_data_long.package_number;
    data.type = printer_data_long.type;
    data.source_address = printer_data_long.target_address;
    data.target_address = printer_data_long.source_address;
    Bambubus_long_package_send(&data);
}
unsigned char s = 0x01;


//===========================================================//
//= 设置、更新耗材信息响应                                    =//
//===========================================================//
/* 数据模板 */
static uint8_t s_update_filament_resp_temp[] = {0x3D, 0xC0, 0x08, 0xB2, 0x08, 0x60, 0xB4, 0x04};
//* 设置、更新耗材信息响应函数 */
void update_filament_info_resp(unsigned char *buf, int length)
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
    /* 发送响应信息至主机 */
    bambu_bus_send_response(s_update_filament_resp_temp, sizeof(s_update_filament_resp_temp));
    /* 设定保存标记，后续在必要时保存数据到Flash。 */
    bambu_bus_need_to_save();
}


bambu_bus_package_type_t BambuBus_run(void)
{
    bambu_bus_package_type_t stu = BambuBus_package_NONE;
    static uint64_t time_set = 0;
    static uint64_t time_motion = 0;

    uint64_t timex = get_time64();

    /*for (auto i : data_save.filament)
    {
        i->motion_set = idle;
    }*/

    if (s_bambu_bus_rev_length)
    {
        int data_length = s_bambu_bus_rev_length;
        s_bambu_bus_rev_length = 0;
        need_debug = false;
        delay(1);
        /* 分析接收的数据，获取包类型。 */
        stu = bambu_bus_get_package_type(s_usart_rev_dump, data_length); // have_data
        switch (stu)
        {
        case BambuBus_package_heartbeat: /* 短包，通信心跳检查 */
        {
            time_set = timex + 1000;
            break;
        }
        case BambuBus_package_filament_motion_short: /* 短包 */
            send_for_motion_short(s_usart_rev_dump, data_length);
            break;
        case BambuBus_package_filament_motion_long: /* 短包 */
            DEBUG_NUM_OUT(s_usart_rev_dump, data_length);
            send_for_motion_long(s_usart_rev_dump, data_length);
            time_motion = timex + 1000;
            break;
        case BambuBus_package_online_detect: /* 短包，设备连线检测 */
            send_for_online_detect(s_usart_rev_dump, data_length);
            break;
        case BambuBus_package_REQx6:    /* 短包 */
            // send_for_REQx6(buf_X, data_length);
            break;
        case BambuBus_long_package_MC_online: /* 长包 */
            send_for_long_packge_MC_online(s_usart_rev_dump, data_length);
            break;
        case BambuBus_longe_package_filament: /* 长包，获取耗材信息 */
            send_for_long_packge_filament(s_usart_rev_dump, data_length);
            break;
        case BambuBus_long_package_version: /* 长包，获取版本信息 */
            send_for_long_packge_version(s_usart_rev_dump, data_length);
            break;
        case BambuBus_package_NFC_detect: /* 短包，NFC检知 */
            // send_for_NFC_detect(buf_X, data_length);
            break;
        case BambuBus_package_set_filament: /* 短包，更新耗材信息 */
        {
            update_filament_info_resp(s_usart_rev_dump, data_length);
            break;
        }
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
        // set_filament_motion(get_now_filament_num(),idle);
        /*for(auto i:data_save.filament)
        {
            i->motion_set=idle;
        }*/
    }
    /* 保存耗材信息至flash */
    if (s_bambu_bus_need_to_save)
    {
        bambu_bus_save_storage_data();
        time_set = get_time64() + 1000;
        s_bambu_bus_need_to_save = false;
    }
    // HAL_UART_Transmit(&use_Serial.handle,&s,1,1000);

    // NFC_detect_run();
    return stu;
}
