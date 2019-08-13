#pragma once

#include <ros/ros.h>

//mission processor includes
#include <object_manager/mission_processor.h>
#include <object_manager/mission_pedestrian.h>
#include <object_manager/mission_traffic_sign.h>
#include <object_manager/mission_none.h>
#include <object_manager/mission_emergency_vehicle.h>
#include <object_manager/mission_accident_vehicle.h>

//sub, pub includes
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/TwistStamped.h>

//c++ libraries
#include <vector>
#include <mutex>

//AVC specific header
#include <object_manager/AVCInterface.h>

enum {
    MISSION_NONE,
    MISSION_PEDESTRIAN,
    MISSION_TRAFFIC_SIGN,
    MISSION_ACCIDENT_VEHICLE,
    MISSION_EMERGENCY_VEHICLE,
    N_MISSION
};

//어떤 미션인지에 따라 하는 행동을 결정하는 class
class MissionHandler{
public:
    // 생성자
    /* 
        std::make_shared는 스마트 포인터로 C++에서는 동적할당된 메모리는 반드시 delete키워드로 해제되어야 한다.
        하지만 일일이 delete하기에는 번거러움이 있어 프로그램의 안전성을 보장하기 위해 스마트 포인터를 제공.
        포인터처럼 동작하는 클래스 템플릿으로 사용이 끝난 메모리를 자동으로 해제해주는 역할을 한다.
    */
    MissionHandler() : mission_list(N_MISSION)
    {   
        /* 
            vector<std::shared_ptr<MissionProcessor> MissionProcessorPtr> mission_list
            결국 mission은 스마트 포인터 자료형을 가지는 vector이므로 
            vector 방 한칸한칸은 스마트포인터이다.
            즉, 첫번째 코드를 해석해보면 MissionProcessorPtr 객체를 가리키는 mission_list[]를 make_shared()함수를 통해 생성한다.
        */
        mission_list[MISSION_NONE]            =
            std::make_shared<NoneProcessor>(); // NoneProcessor는 MissionProcessor class의 자식 class로 processMission 멤버함수를 가진다.
        mission_list[MISSION_PEDESTRIAN]      = 
            std::make_shared<PedestrianProcessor>(); //PedestrianProcessor는 MissionProcessor class의 자식 class로 
                                                      // processMission() 멤버함수를 가지며, 보행자를 출력하는 함수의 정의가 있다.
        mission_list[MISSION_TRAFFIC_SIGN]    = 
            std::make_shared<TrafficSignProcessor>(); // TrafficSignProcessor는 MissionProcessor calss의 자식 class로 processMission 멤버함수를 가진다.
                                                      // 신호등 구간을 처리하는 함수이다.
        mission_list[MISSION_ACCIDENT_VEHICLE]    = 
            std::make_shared<AccidentVehilceProcessor>(); // 사고차량구간을 처리하는 함수가 정의되어있다.
        mission_list[MISSION_EMERGENCY_VEHICLE]    = 
             std::make_shared<EmergencyVehicleProcessor>(); // 응급차량 구간을 처리하는 함수가 정의되어있다.

        //mission_list[MISSION_NONE]            =// estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. 
        //     std::make_shared<AccidentVehilceProc// estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. essor>();
        // mission_list[MISSION_PEDESTRIAN]      = // estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. 
        //     std::make_shared<AccidentVehilceProc// estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. essor>();
        // mission_list[MISSION_TRAFFIC_SIGN]    = 
        //     std::make_shared<AccidentVehilceProcessor>();
        // mission_list[MISSION_ACCIDENT_VEHICLE]    = 
        //     std::make_shared<AccidentVehilceProcessor>();
        //mission_list[MISSION_EMERGENCY_VEHICLE]    = 
        //    std::make_shared<NoneProcessor>();        

    }
    // mission을 수행시키는 함수.
    void run(AVCInterface& avc_interface,
        int mission_identifier, 
        const object_manager_msgs::combined msg){

        //set current mission processor
        MissionProcessorPtr cur_mission_ptr = mission_list[mission_identifier];
        
        //process mission specific workings and set twist filter
        cur_mission_ptr->processMission(avc_interface, msg);
    }

    //사용자가 정의한 set_param 함수.
    void set_param(object_manager::ObjectManagerConfig &config){
        for(auto mission_ptr : mission_list)
            mission_ptr->set_param(config);
        // 각각의 mission의 set_param함수에 config 파일을 넣어준다.
    }

private:
    ros::NodeHandle nh, private_nh;
    std::vector<MissionProcessorPtr> mission_list;
    /* typedef std::shared_ptr<MissionProcessor> MissionProcessorPtr;
    class MissionProcessor{
        public:
            MissionProcessor()  {}
        //두개의 멤버 함수.
        // virtual 키워드 : 가상함수 선언, 동적바인딩이 된다. 즉, 호출 시에 함수의 주소가 정해진다. 오버라이딩이 가능해진다. 
        // 상속관계에서 자식의 함수를 원하는 대로 사용하고 싶다면 virtual 키워드를 사용해야한다.
        virtual void processMission(AVCInterface& avc_interface, const object_manager_msgs::combined msg) = 0;
        virtual void set_param(object_manager::ObjectManagerConfig &config){}
    };
    */
};