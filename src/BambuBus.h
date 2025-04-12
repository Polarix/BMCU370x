#pragma once

#include "main.h"

#define BAMBU_BUS_VER 5
#define BAMBU_BUS_FILAMENT_NUM_MAX 4

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum _filament_state_
    {
        offline,
        online,
        NFC_waiting
    }filament_state_t;

    typedef enum _filament_motion_set_
    {
        need_pull_back,
        need_send_out,
        on_use,
        idle
    }filament_motion_set_t;

    typedef enum _bambu_bus_package_type_
    {
        BambuBus_package_ERROR = -1,
        BambuBus_package_NONE = 0,
        BambuBus_package_filament_motion_short,
        BambuBus_package_filament_motion_long,
        BambuBus_package_online_detect,
        BambuBus_package_REQx6,
        BambuBus_package_NFC_detect,
        BambuBus_package_set_filament,
        BambuBus_long_package_MC_online,
        BambuBus_longe_package_filament,
        BambuBus_long_package_version,
        BambuBus_package_heartbeat,
        BambuBus_package_ETC,
        __BambuBus_package_packge_type_size
    }bambu_bus_package_type_t;

    typedef enum _bambu_bus_device_type_
    {
        BambuBus_none=0x0000,
        BambuBus_AMS=0x0700,
        BambuBus_AMS_lite=0x1200,
    }bambu_bus_device_type_t;

    void bambu_bus_init(void);
    bambu_bus_package_type_t bambu_bus_ticks_handler(void);
    bool bambu_bus_load_config(void);
    void bambu_bus_save_config_later(void);
    void bambu_bus_save_config_now(void);
    int bambu_bus_get_filament_num(void);
    bambu_bus_device_type_t bambu_bus_get_device_type();
    extern void bambu_bus_add_filament_meters(int num, float meters);
    extern float bambu_bus_get_filament_meters(int num);
    void bambu_bus_set_filament_online_state(int num, bool if_online);
    bool bambu_bus_filament_is_online(int num);
    filament_motion_set_t bambu_bus_get_filament_motion_set(int num);
    extern void set_filament_motion(int num, filament_motion_set_t motion);

    #ifdef __cplusplus
}
#endif