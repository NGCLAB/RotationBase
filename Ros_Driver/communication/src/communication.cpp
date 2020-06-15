#include <ros/ros.h>
#include <serial/serial.h>
#include <time.h>
#include <ctime>
#include <sys/sysinfo.h>

using namespace std;
//更新消息
bool updateMessage = false;

class toNav
{
public:
    toNav();
    int init_serial();
    void get_data();
    void reconnected()
    {
        ros::Rate r(1);
        while(1)
        {
            if(init_serial() == 1)
            {
                ROS_INFO("port to navigation reconnected");
                break;
            }
            r.sleep();
        }
    }

    uint8_t navcnt;

    uint8_t emptyBuff[70];

    serial::Serial ros_serial;
    ros::Publisher send_pub;
    ros::Publisher utc_pub;
    ros::Publisher gps_pub;
    ros::Publisher imu_pub;

    ros::Subscriber pertonav_sub;

    uint8_t send_buff[100];  //发送缓存数组
    uint8_t receive[2];
    uint8_t valid_data[100];  //有效数据接收缓存数组
};
toNav::toNav()
{
    ros::NodeHandle nh("~");

}


int toNav::init_serial()
{
    std::string port = "/dev/ttyUSB1";
    int baud = 115200;
    try
    {
        //设置串口属性，并打开串口
        ros_serial.setPort(port);  //端口
        ros_serial.setBaudrate((uint32_t)baud);  //波特率
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        ros_serial.setTimeout(to);
        if(!ros_serial.isOpen())
          ros_serial.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("serial port "<< port << " disconnected, try every second.");
        return -1;
    }
    //检测串口是否已经打开，并给出提示信息
    if(ros_serial.isOpen())
    {
        ROS_INFO_STREAM("Serial Port" <<port<<" initialized");
        ros_serial.flush();
        return 1;
    }
    else
    {
        return -1;
    }
}



void toNav::get_data()
{
    if(ros_serial.available())
    {
        memset(receive, 0, 2);  //帧头数组清零

        ros_serial.read(receive,1);  //从串口读取一个字节

        if(receive[0] == 0xAA)
        {
            ros_serial.read(receive,1);

            //连续两次读到帧头0XAA
            if(receive[0] == 0xAA)
            {
                ros_serial.read(receive,2);  //再读两个字节

                uint8_t data_len = receive[1];  //获取此数据帧的长度

                ros_serial.read(valid_data, data_len+1);  //从串口读取此数据帧剩余数据

                uint8_t check_sum = (0xAA + 0xAA + receive[0] + receive[1]);  //最后一个字节是校验和

                // 计算校验和
                for(int i=0; i<data_len; i++)
                {
                    check_sum += valid_data[i];
                }
                //ROS_INFO("Len:%d,sum:%d\n", (int)data_len,check_sum);
                if( check_sum == valid_data[data_len] )
                {
                    uint8_t cnt = 0;
                    int data_tmp;

                    // 获取角度数据
                    data_tmp = (valid_data[cnt++]<<24) | (valid_data[cnt++]<<16) | (valid_data[cnt++]<<8) | valid_data[cnt++];
                   // msg.data = (int16_t)data_tmp;
                    cout<<"angle: "<<data_tmp<<endl;
                    //angle_pub.publish(data_tmp);

                }
            }
        }
    }
}



int main(int argc, char *argv[])
{
  ros::init(argc,argv,"communication");
  ros::NodeHandle nh_p;
  toNav m_navobj;
  m_navobj.init_serial();
  ros::Rate rt(50);
  uint8_t find_flag = 0;
  uint8_t last_second, now_second;
  while(ros::ok())
  {
    m_navobj.get_data();
    rt.sleep();
    ros::spinOnce();
  }
  m_navobj.ros_serial.close();

  return 0;
}



