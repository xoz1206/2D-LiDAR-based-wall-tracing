#pragma once
#include <object_manager/mission_processor.h>

class PedestrianProcessor : public MissionProcessor{
public:
    // 미션을 처리하는 함수.
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){

        auto& detectedObj = msg.obj_ary;
        bool isPersonMsg = false;

        // 사람이 검출되었는지 확인하는 부분.
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if(it->label == "person") {
                isPersonMsg = true;                                
                break;
            }
        }
        // 사람이 검출되지 않았으면 더이상 진행할 필요없다.
        if(!isPersonMsg) return;

        // 사람이 검출되었을 때..
        ROS_WARN("[MISSION 1] Person msg detected");
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            if((*it).label == "person" && (*it).pose.position.x <= pedestrian_stop_distance_threshold
                && it->pose.position.y >= pedestrian_right_distance_threshold 
                && it->pose.position.y <= pedestrian_left_distance_threshold ){
                ROS_WARN("[MISSION 1] STOP!");
                // 멈춤.
                avc_interface.enableEstop(true);
                avc_interface.publish();
                
                return;
            }
        }        
        // 사람이 없을 때 다시 출발.
        ROS_WARN("[MISSION 1] GO!");
        avc_interface.enableEstop(false); 
        avc_interface.publish();
        return;
    }

    
    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
        pedestrian_stop_distance_threshold = config.pedestrian_STOP_distance_threshold; 
        pedestrian_right_distance_threshold = config.pedestrian_RIGHT_distance_threshold; 
        pedestrian_left_distance_threshold = config.pedestrian_LEFT_distance_threshold; 
    }
private:
    // 사람과의 거리에 대한 threshold
    double pedestrian_stop_distance_threshold = 30.0;
    double pedestrian_right_distance_threshold = -0.9;
    double pedestrian_left_distance_threshold = 4.55;
};