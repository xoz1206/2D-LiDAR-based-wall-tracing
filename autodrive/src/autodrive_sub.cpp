#include <ros/ros.h>
#include <serial/serial.h>
#include <deque>
#include <thread>
#include <mutex>
#include "autodrive/autodrive.h"

#define TX_PACKET_LENGTH 14
#define TX_SERIAL_FREQUENCY 10
#define TX_STOP_CHECK_PERIOD 10

/*
 실제 차량의 dynamic에 의해 제한되는 최대조향각과 최대가속도, 최대속도값은 teb_local_planner의 parameter 튜닝으로 설정함.
 여기서는 Boundary Check만 진행 (원래 여기에 걸리는 것도 문제가 있음)
 여기서 걸리지 않게 테스트하면서 teb_local_planner parameter 튜닝할 필요가 있지만, 이 작업은 시간이 있으면 진행하는게 좋을듯
*/

uint8_t packet[TX_PACKET_LENGTH] = {};  //플랫폼에 패킷을 보낼때 받을때와는 다르게 14바이트로 보넴 packet[14]

//serial
serial::Serial *ser;
std::mutex lock;

void initTx(const ros::NodeHandle& nh){
    packet[0] = static_cast<uint8_t>(0x53);
    packet[1] = static_cast<uint8_t>(0x54);
    packet[2] = static_cast<uint8_t>(0x58);
    //manual OR auto. 0x00 - manual/0x01 - auto
    packet[3] = static_cast<uint8_t>(0x01);
    //estop. 0x00 - off/0x01 - on
    packet[4] = static_cast<uint8_t>(0x00);

    packet[12] = static_cast<uint8_t>(0x0D);//0x0D
    packet[13] = static_cast<uint8_t>(0x0A);//0x0A
}

void serialWrite(){ // thread tr이 수행하는 함수.
    ros::Rate loop_rate(TX_SERIAL_FREQUENCY);
    uint8_t alive = 0; 
    while(true){
        packet[11] = static_cast<uint8_t>(alive); 
        alive = (alive + 1) % 256; // 0~255 alive 값을 보내주기 위한 식. 한번 write할때마다 + 1
        ROS_INFO("alive : %d", packet[11]);
        ser->write(packet,TX_PACKET_LENGTH); //packet을 보내는 함수
        loop_rate.sleep();
    }
}

void createSerialPacket(const autodrive::autodrive::ConstPtr& msg){
/*  
    GEAR
    0x00 : forward
    0x01 : neutral
    0x02 : backward
*/
    packet[5] = static_cast<uint8_t>(1); //gear;

    float angle = - msg->goto_radian; //   checkSteeringBound(angle);
    uint16_t serialspeed = 30;
    *(uint16_t*)(packet + 7) = serialspeed;
    ROS_INFO("serial speed : %d", serialspeed);
    int16_t serialSteeringAngle;

    if(msg->goto_radian == 0.0){serialSteeringAngle = 0;}
    else if(msg->goto_radian >= -2.0 && msg->goto_radian <= 2.0){serialSteeringAngle = angle * 103.9 * 0.25;}
    else { serialSteeringAngle = angle * 103.9 * 0.5;}

    if(serialSteeringAngle >= 40.0){
        *(int8_t*)(packet + 4) = *(int8_t*)(1);
        ROS_INFO("E-STOP!!");
    }

    *(int8_t*)(packet + 8) = *((int8_t*)(&serialSteeringAngle) + 1);
    *(int8_t*)(packet + 9) = *(int8_t*)(&serialSteeringAngle);
    ROS_INFO("serial angle : %d", serialSteeringAngle);
    packet[10] = static_cast<uint8_t>(1);
    ROS_INFO("serial brake : %u", 1);
}



void CallBack(const autodrive::autodrive::ConstPtr& msg){
    createSerialPacket(msg);
}



int main(int argc, char *argv[]){
    if(argc < 2){
        ROS_ERROR("give me [path]");
        return -1;
    }
//open serial=============================================================================================
    ser = new serial::Serial(); // serial::Serial* ser = new serial::Serial();
    ser->setPort(argv[1]);
    ser->setBaudrate(115200); // platform 기본 시리얼 설정
    serial::Timeout to = serial::Timeout::simpleTimeout(1000); // platform 기본 시리얼 설정
    ser->setTimeout(to); // platform 기본 시리얼 설정
    ser->open(); //serial 연결
    if(!ser->isOpen()) throw serial::IOException("ser.isOpen() error!",__LINE__,"ser.isOpen() error!");
    ROS_INFO("serial setting done");
//========================================================================================================
    autodrive::autodrive msg;
    ros::init(argc, argv, "autodrive_sub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("opencv_msg", 1000, &CallBack);
    initTx(nh);
    
    std::thread tr(serialWrite);
    tr.detach();

    ros::spin();
    //5hz->9.75
    //10hz->9.95
    //15hz->9.56
    //20hz->9.68
    //50hz->9.02

}