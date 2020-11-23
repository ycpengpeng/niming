//
// Created by pengpeng on 10/1/20.
//
/****************************************************************************
 *用串口读取匿名光流的数据
 *串口：ttyS2=TELEM2=USART3
 *


 ****************************************************************************/
#include <px4_config.h>
#include <px4_defines.h>
#include "niming.h"







niming_parse::niming_parse(): ModuleParams(nullptr)
{
/*    parameters_updated();
    _msg_sum_chk = 0x00;
    memset(&_vision_position, 0 , sizeof(_vision_position));*/
}

int
niming_parse::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("niming",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);


    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

niming_parse* niming_parse::instantiate(int argc, char *argv[])
{
    niming_parse *instance = new niming_parse();   //在堆中划出一块空间，里面存放一个niming_parse的对象， 返回指向这块内存的指针

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

int niming_parse::custom_command(int argc, char **argv)
{
    return print_usage("unknown command");
}

int
niming_parse::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
            R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

Description
This is a module used to parse data from ni_ming optical flow through serial port.


)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("module", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
    PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}


/*void niming_parse::parameters_update_poll()   //这个函数使用来检测串口的配置参数是否改变，如果改变就进行更新  猜测：parameter_update这个话题的内容就是关于参数是否改变
{
    parameter_update_s param_upd;  //parameter_update_s 这个数据类型对应了 parameter_update.msg这个消息类型，在编译时由这个消息文件自动生成了parameter_update_s这种数据类型
    if (_params_sub.update(&param_upd)) {   //检查param_upd这个变量是否由更新
        updateParams();
        parameters_updated();
    }
}*/


void niming_parse::quaternion2euler(float q[4],euler &ang)
{
    ang.roll=atan(2*(q[0]*q[1]+q[2]*q[3])/(1-2*(q[1]*q[1]+q[2]*q[2])));
    ang.pitch=2*(q[0]*q[2]-q[1]*q[3]);
    ang.yaw=atan(2*(q[0]*q[3]+q[1]*q[2])/(1-2*(q[2]*q[2]+q[3]*q[3])));
}



 void niming_parse::run()
{
    int32_t i=0;
    i=i;
    unsigned char sum_check=0;
    unsigned char add_check=0;
    add_check=add_check;
    int32_t height=0;

    int16_t vx1=0;
    vx1*=1;
    int16_t vx2=0;
    vx2*=1;
    int16_t vy1=0;
    vy1*=1;
    int16_t vy2=0;
    vy2*=1;
    float q[4];
    float z_last=0.f;

    uint64_t current_time=hrt_absolute_time();
    uint64_t current_time2=hrt_absolute_time();
    uint64_t delta_time;

    uint64_t delta_time2=hrt_absolute_time();


	matrix::Eulerf angle(0,0,0);

    //int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));

    //struct vehicle_attitude_s vehicle_attitude;

    _niming_data_pub=orb_advertise(ORB_ID(vehicle_visual_odometry), &_niming_data);
    matrix::Quaternion<float> _q(0,1,0,0);  //从北东地到北西天

    serial_init();
    //orb_set_interval(vehicle_attitude_sub, 10);


    while (!should_exit())
    {
        static unsigned char buffer[30] = {0};

        //orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &vehicle_attitude);

        // angle = matrix::Quatf(vehicle_attitude.q);
        // angle=angle;

        buffer[0] = parse_msg_type<unsigned char>();
        if (buffer[0] == 0xAA)
        {
            buffer[1] = parse_msg_type<unsigned char>();
            if (buffer[1] == 0XFF)
            {
               // PX4_INFO("GET FRAME HEAD");
                buffer[2] = parse_msg_type<unsigned char>();   //ID

                if(buffer[2]==0X51)   //光流信息帧
                {

                    buffer[3]=parse_msg_type<unsigned char>();//LEN
                    buffer[4]=parse_msg_type<unsigned char>();  //MODE
                    if(buffer[4]==0x02)  //MODE 2
                    {
						//PX4_INFO("MODE 2 READ");
						buffer[5]=parse_msg_type<unsigned char>();  //STATE
                        buffer[6]=parse_msg_type<unsigned char>();  //DX2_1
                        buffer[7]=parse_msg_type<unsigned char>();   //DX2_2
                        buffer[8]=parse_msg_type<unsigned char>();   //DY2_1
                        buffer[9]=parse_msg_type<unsigned char>();      //DY2_2
                        buffer[10]=parse_msg_type<unsigned char>();  //DXFIX_1
                        buffer[11]=parse_msg_type<unsigned char>();  //DXFIX_2
                        buffer[12]=parse_msg_type<unsigned char>();  //DYFIX_1
                        buffer[13]=parse_msg_type<unsigned char>();  //DYFIX_2
                        buffer[14]=parse_msg_type<unsigned char>();  //INTX_1
                        buffer[15]=parse_msg_type<unsigned char>();  //INTX_2
                        buffer[16]=parse_msg_type<unsigned char>();  //INTY_1
                        buffer[17]=parse_msg_type<unsigned char>();  //INTY_2
                        buffer[18]=parse_msg_type<unsigned char>();  //QUALITY
                        buffer[19]=parse_msg_type<unsigned char>();  //SUM_CHECK
                        buffer[20]=parse_msg_type<unsigned char>();  //ADD_CHECK

                        sum_check=0;
						add_check=0;
                        for (int k=0;k<=18;k++)
                        {
                            sum_check+=buffer[k];
							add_check+=sum_check;
                        }
                        if((sum_check==buffer[19])&&(add_check==buffer[20]))
                        {
                            vx1=(int16_t)((buffer[7]<<8)|buffer[6]);   //
                            vy1=-(int16_t)((buffer[9]<<8)|buffer[8]);   //
                            vx2=(int16_t)((buffer[11]<<8)|buffer[10]);
                            vy2=-(int16_t)((buffer[13]<<8)|buffer[12]);

                            //_niming_data.vx=vx1*cos(angle.psi())/100.0-vy1*sin(angle.psi())/100.0;
                            //_niming_data.vy=vy1*cos(angle.psi())/100.0+vx1*sin(angle.psi())/100.0;

                            // delta_time=hrt_absolute_time()-current_time;
                            // _niming_data.x=_niming_data.x+delta_time/1000000.f*(vx2*(float)cos(angle.psi())-vy1*(float)sin(angle.psi()))/100.f;
                            // _niming_data.y=_niming_data.y+delta_time/1000000.f*(vy2*(float)cos(angle.psi())+vx1*(float)sin(angle.psi()))/100.f;
                            // current_time=hrt_absolute_time();



                            //PX4_INFO("DELAT TIME %ld",(long int)delta_time);

                        }
                    }
                }
                else if(buffer[2]==0X34)   //测距信息帧
                {
                    buffer[3]=parse_msg_type<unsigned char>();//LEN
                    buffer[4]=parse_msg_type<unsigned char>();  //direction
                    buffer[5]=parse_msg_type<unsigned char>();  //angle1
                    buffer[6]=parse_msg_type<unsigned char>();  //angle2
                    buffer[7]=parse_msg_type<unsigned char>();   //dist1
                    buffer[8]=parse_msg_type<unsigned char>();   //dist2
                    buffer[9]=parse_msg_type<unsigned char>();      //dist3
                    buffer[10]=parse_msg_type<unsigned char>();  //dist4
                    buffer[11]=parse_msg_type<unsigned char>();  //和校验
                    buffer[12]=parse_msg_type<unsigned char>();  //附加校验

                    sum_check=0;
					add_check=0;
                    for (int k=0;k<=10;k++)
                    {
                        sum_check+=buffer[k];
						add_check+=sum_check;

                    }
					if((sum_check==buffer[11])&&(add_check==buffer[12]))
                    {
                        height=(buffer[10]<<24)+(buffer[9]<<16)+(buffer[8]<<8)+(buffer[7]);
                        _niming_data.z=(float)height/100.f;  //高度，单位m
						_niming_data.z=-(double)_niming_data.z*cos(angle.phi())*cos(angle.theta());

                        delta_time2=hrt_absolute_time()-current_time2;

                        _niming_data.vz=(_niming_data.z-z_last)/(delta_time2/1000000.f);
                        z_last=_niming_data.z;
                        current_time2=hrt_absolute_time();

                    }

                }
                else if(buffer[2]==0x04)   //四元数姿态帧
                {
                    buffer[3]=parse_msg_type<unsigned char>();//LEN
                    buffer[4]=parse_msg_type<unsigned char>();//V0_1
                    buffer[5]=parse_msg_type<unsigned char>();//V0_2
                    buffer[6]=parse_msg_type<unsigned char>();//V1_1
                    buffer[7]=parse_msg_type<unsigned char>();//V1_2
                    buffer[8]=parse_msg_type<unsigned char>();//V2_1
                    buffer[9]=parse_msg_type<unsigned char>();//V2_2
                    buffer[10]=parse_msg_type<unsigned char>();//V3_1
                    buffer[11]=parse_msg_type<unsigned char>();//V3_2
                    buffer[12]=parse_msg_type<unsigned char>();//FUSION_STA
                    buffer[13]=parse_msg_type<unsigned char>();//SUM_CHECK
                    buffer[14]=parse_msg_type<unsigned char>();//ADD_CHECK

                    sum_check=0;
                    add_check=0;
                    for (int k=0;k<=12;k++)
                    {
                        sum_check+=buffer[k];
                        add_check+=sum_check;
                    }
                    if((sum_check==buffer[13])&&(add_check==buffer[14]))
                    {
                        q[0]=(int16_t)((buffer[5]<<8)|buffer[4])/10000.f;

                        q[1]=(int16_t)((buffer[7]<<8)|buffer[6])/10000.f;
                        q[2]=(int16_t)((buffer[9]<<8)|buffer[8])/10000.f;
                        q[3]=(int16_t)((buffer[11]<<8)|buffer[10])/10000.f;

                        matrix::Quaternion<float> qw(q);
                        qw=_q*qw*(_q.inversed());

                        _niming_data.q[0]=qw(0);
                        _niming_data.q[1]=qw(1);
                        _niming_data.q[2]=qw(2);
                        _niming_data.q[3]=qw(3);

                        angle = qw;

                    }
                }

                _niming_data.vx=vx1*cos(angle.psi())/100.0-vy1*sin(angle.psi())/100.0;
                _niming_data.vy=vy1*cos(angle.psi())/100.0+vx1*sin(angle.psi())/100.0;
                delta_time=hrt_absolute_time()-current_time;
                _niming_data.x=_niming_data.x+delta_time/1000000.f*(vx2*(float)cos(angle.psi())-vy1*(float)sin(angle.psi()))/100.f;
                _niming_data.y=_niming_data.y+delta_time/1000000.f*(vy2*(float)cos(angle.psi())+vx1*(float)sin(angle.psi()))/100.f;
                current_time=hrt_absolute_time();


                _niming_data.local_frame=0;
                _niming_data.timestamp=hrt_absolute_time();

                _niming_data.pose_covariance[0]=NAN;
                _niming_data.pose_covariance[15]=NAN;

                orb_publish(ORB_ID(vehicle_visual_odometry), _niming_data_pub, &_niming_data);

                // time++;
                // time=0;

                // if(time>=100)
                // {

                //     time=0;
                // }

            }

        }
    }
}




/*
bool niming_parse::parse_frame_head(u_char limit)   //读取校验数据帧的帧头
{

    uint8_t count = 0;


    while(count<limit)
    {

        _data[0]=parse_msg_type<u_char>();

        if(_data[0] == FRAME_HEAD1)
        {
            PX4_INFO("PARSE HEAD1");

            _data[1]=parse_msg_type<u_char>();
            if(_data[1] == FRAME_HEAD2)
            {
                PX4_INFO("PARSE HEAD2");
                return true;

            }
        }
        count++;
        //PX4_INFO("PARSE HEAD");
    }
    return false;
}
*/




void niming_parse::get_data()
{
    _cdata_buffer = '0';

    auto _a =  read(_serial_fd,&_cdata_buffer,1); //从串口读取一个字节
    _a = _a;
}



template <typename T>
T niming_parse::parse_msg_type()
{
    get_data();
    return (T)_cdata_buffer;
}




int niming_parse::print_status()
{
    PX4_INFO("Running");
    // TODO: print additional runtime information about the state of the module

    return 0;
}




//串口参数配置函数,配置成功返回True
bool niming_parse::serial_init()
{
    _serial_fd = open("/dev/ttyS1", O_RDWR | O_NOCTTY);   //ttyS2=TELEM2=USART3 ， ttyS1=TELEM1=USART2，  open函数返回文件描述符(file descriptor)
    if (_serial_fd < 0) {
        PX4_INFO( "failed to open port: /dev/ttyS%d",_ser_com_num);
        return false;
    }
    int speed=0;
    //speed=1;  如果不用一下speed这个变量，编译会报错？
    switch (_ser_buadrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 256000: speed = B256000; break;
        case 500000: speed = B500000; break;
        case 576000: speed = B576000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        default:
            PX4_INFO("ERR: baudrate: %d\n", _ser_buadrate);
            return -EINVAL;
    }

    struct termios uart_config;
    int termios_state;
    tcgetattr(_serial_fd, &uart_config);
    uart_config.c_oflag &= ~ONLCR;  //如果设置ONLCR，在发送换行符（'/n'）前先发送回车符（'/r'）
    uart_config.c_cflag &= ~(CSTOPB|PARENB); // 如果设置CSTOPB，则会在每个数据帧后产生两个停止位;如果设置PARENB，会产生一个奇偶检验位：
    uart_config.c_cflag &= ~CRTSCTS;  //不使用流控制
    if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
        PX4_INFO("ERR: %d (cfsetispeed)\n", termios_state);
        return false;
    }
    if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
        PX4_INFO("ERR: %d (tcsetattr)\n", termios_state);
        return false;
    }
    PX4_INFO("niming serial init success Telem%d",_ser_com_num);
    return true;


}



int niming_main(int argc, char *argv[])
{
    return niming_parse::main(argc, argv);
}
