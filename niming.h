//
// Created by pengpeng on 10/1/20.
//
/****************************************************************************
 *用串口读取匿名光流的数据
 *串口：ttyS2=TELEM2=USART3 ttyS2=TELEM2=USART3 ， ttyS1=TELEM1=USART2
 *


 ****************************************************************************/
#ifndef PX4_NIMING_H
#define PX4_NIMING_H



#pragma once
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <px4_module.h>
#include <cstring>
#include <cerrno>
#include <cmath>
#include <drivers/drv_hrt.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/math.hpp>
#include <matrix/matrix/Quaternion.hpp>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <px4_defines.h>
#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <matrix/math.hpp>
#include <px4_module_params.h>
#include <uORB/Publication.hpp>
//#include <uORB/PublicationMulti.hpp>
//#include <uORB/PublicationQueued.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/commander_state.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/landing_gear.h>
#include <px4_log.h>
#include <px4_posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>


#define FRAME_HEAD1 0XAA      //数据帧 帧头
#define FRAME_HEAD2 0XFF     //数据帧 第二个字节


float multibytes2one(float a, float b);
float multibytes2one(float a, float b,float c,float d);


extern "C" __EXPORT int niming_main(int argc, char *argv[]);

typedef struct euler_angle
{

    float yaw;
    float pitch;
    float roll;
}euler;

class niming_parse: public ModuleBase<niming_parse>, public ModuleParams
{
public:
    niming_parse();
    virtual ~niming_parse()=default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    static niming_parse *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

    bool serial_init();

    void parameters_update_poll();

    bool parse_frame_head(u_char limit);

    void get_data();

    void quaternion2euler(float q[4],euler &ang);

    template <typename T>
    T parse_msg_type();


private:

    int      _params_sub=orb_subscribe(ORB_ID(parameter_update));


    int     _ser_com_num{1};    //串口：ttyS2=TELEM2=USART3 ttyS1=TELEM1=USART2
    int     _ser_buadrate{256000};   //串口通信波特率
    int     _serial_fd{-1};           //串口通信fd, file descriptor
    u_char    _cdata_buffer{0};   //
    u_char      _id_byte{0};      //数据帧的ID
    u_char      _len_byte{0};  //数据帧的LEN字节
    //  u_char      _data[30];



    u_char     _msg_sum_chk{0};   //数据帧的和校验位
    u_char      _ability_byte{0}; //数据帧的功能字

    u_char      _mode_byte{0};    //数据帧的MODE
    u_char      _state_byte{0};  //数据帧的STATE


    orb_advert_t                 _niming_data_pub{nullptr};   //定义一个话题句柄

    vehicle_odometry_s               _niming_data{};    //optical_flow话题类型的变量，用来存储数据


};


#endif //PX4_NIMING_H
