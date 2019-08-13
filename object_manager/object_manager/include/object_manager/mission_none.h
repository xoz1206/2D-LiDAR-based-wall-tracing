#pragma once
#include <object_manager/mission_processor.h>
#include <object_manager_msgs/combined.h>

class NoneProcessor : public MissionProcessor{
public:
    NoneProcessor()
    {}
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){
        avc_interface.publish();// estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. 
        return;
    }
private:
};