#pragma once
#include <ros/ros.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <memory>
#include <object_manager/ObjectManagerConfig.h>
#include <object_manager/AVCInterface.h>
class MissionProcessor{
public:
    MissionProcessor()
    {}
    //???? 무슨 코드??
    virtual void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg) = 0;
    /* Process mission specific works and decide if vehicle stops */ 
    virtual void set_param(object_manager::ObjectManagerConfig &config){}
};

typedef std::shared_ptr<MissionProcessor> MissionProcessorPtr;