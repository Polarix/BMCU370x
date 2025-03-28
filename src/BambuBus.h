#pragma once

#include "main.h"

#define BAMBU_BUS_VER 5
#define BAMBU_BUS_FILAMENT_NUM 4

#ifdef __cplusplus
extern "C"
{
#endif

    enum _filament_status
    {
        offline,
        online,
        NFC_waiting
    };
    enum _filament_motion_state_set
    {
        need_pull_back,
        need_send_out,
        on_use,
        idle
    };

    typedef enum _e_bambu_bus_package_type_
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

    typedef enum _e_bambu_bus_dev_type_
    {
        BambuBus_none=0x0000,
        BambuBus_AMS=0x0700,
        BambuBus_AMS_lite=0x1200,
    }bambu_bus_dev_type_t;

    extern void BambuBus_init();
    extern bambu_bus_package_type_t BambuBus_run(void);

    bool bambu_bus_load_storage_data(void);
    void bambu_bus_need_to_save(void);
    extern int get_now_filament_num();
    extern uint16_t get_now_BambuBus_device_type();
    extern void reset_filament_meters(int num);
    extern void add_filament_meters(int num, float meters);
    extern float get_filament_meters(int num);
    extern void set_filament_online(int num, bool if_online);
    extern bool get_filament_online(int num);
    _filament_motion_state_set get_filament_motion(int num);
    extern void set_filament_motion(int num, _filament_motion_state_set motion);
    extern bool BambuBus_if_on_print();
#ifdef __cplusplus
}
#endif