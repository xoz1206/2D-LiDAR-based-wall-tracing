#pragma once
#include <object_manager/mission_processor.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>
#include <object_manager/genAutowareObstacles.h>

constexpr static double MISSION5_VELOCITY_MPH = 8.61;// 31 kph

class EmergencyVehicleProcessor : public MissionProcessor{
public:
    //생성자.
    EmergencyVehicleProcessor() : private_nh("~"), e_car_poly_gap(0.5)
    {
        std::vector<double> section_tmp;
        //emergency_critical_section: [-325.598, -1.13492, -326.014, 6.63827, -22.9755, 6.11816, -23.5374, -1.55422]
        if (!private_nh.getParam("emergency_critical_section", section_tmp))
            throw std::runtime_error("set emergency_critical_section!");
        
        section_poly.points.resize(4);
        // 4개의 점 저장.
        for(int i = 0 ; i < 4; ++i){
            section_poly.points[i].x = section_tmp[2*i];
            section_poly.points[i].y = section_tmp[2*i + 1];
        }
    }
    //미션을 수행하는 함수.
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){
        
        // 기술 교류회때 응급차량 테스트시 미션 수행 이후 패트롤에 의한 오작동 발생시 한번만 수행하게 하자.
        auto& detectedObj = msg.obj_ary;

        bool processMission = false;

        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            
            // lidar를 이용한 검출. 
            if(it->label == "AMB" && (!emergency_car_appeared)){
                processMission = true;
                break;
            }
            // 차량을 감지하고, 응급차량이 나타났으며, 그것에 대한 polygon이 존재한다면 응급차량을 찾은 것으로 판단.
            if(it->label == "car" && emergency_car_appeared && it->convex_hull.polygon.points.size()){
                processMission = true;
                break;
            }
        }
        if (!processMission) return;
        // 응급차량이 뒤에서 오고있을 때(라이더 검출)), emergency_critical_section를 만들어 차선을 바꿔줄 때 사용하는 코드.
        if(!emergency_car_appeared){ // AMB was not detected before            
            for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
                if(it->label == "AMB" && (-it->pose.position.x <= AMB_start_distance)){ // const double AMB_start_distance = 10.0;
                    emergency_car_appeared = true; //응급차량 발견한 것으로 판단.
                    
                    double new_x = avc_interface.getCurpos().pose.position.x + 20; // 자동차의 현재 위치 값을 가져온다. 
                    if (new_x < section_poly.points[2].x) { // -42 보다 작은 현재 위치가 나와야 가능. // -22
                        makeFakebox = true; // 가짜 box 생성.
                        //왼쪽 box 라인인듯. 
                        section_poly.points[0].x = new_x + 60; // 사선으로 움직이도록 box 설정.
                        section_poly.points[1].x = new_x;
                    }
                    else makeFakebox = false;

                    amb_detected_time = ros::Time::now();
                    break;
                }   
            }
        }
        // 응급차량이 지나갔을 때 (카메라))
        else{ // AMB is located nearby
            double dt = (ros::Time::now() - amb_detected_time).toSec(); // 검출된 후의 시간을 체크한다.
            if (dt > 3){ // 검출된지 3초초과로 지났을 경우.
                for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
                    // 응급차량이 어느정도 지나갔다면 emergency_critical_section을 제거하고 emergency_car_appeared를 false로 바꿔준다.                
                    if( it->label == "car" && it->pose.position.x >= AMB_escape_distance ){
                        if (!it->convex_hull.polygon.points.size()) break;                        
                        emergency_car_appeared = false;
                    }
                }
            }
        }
        // 응급차량이 나타났을 때 (뒤)
        if (emergency_car_appeared){
            if (makeFakebox){
                autoware_msgs::DetectedObject obs;
                genObstacleMsgFromPolygon(obs, section_poly, 0.5); // 0.5 간격으로 critical_section의 점을 찍는다.
                avc_interface.setObstacle("AMB", obs); // AMB라는 이름의 장애물을 만든다.
                
                avc_interface.enableControlTwist(true); // 차선을 바꾼다.
                //constexpr static double MISSION5_VELOCITY_MPH = 8.61;// 31 kph
                avc_interface.setGoalVelocity(MISSION5_VELOCITY_MPH);//not use omega // 속도를 변경한다. 
            }
        } else{ // 응급차량이 없거나 지나갔을 때
            avc_interface.eraseObstacle("AMB"); // 장애물 제거
            avc_interface.enableControlTwist(false);
        }
        avc_interface.publish();

    }

    void set_param(object_manager::ObjectManagerConfig &config)
    {
    }
private:
    ros::Publisher e_vehicle_debug_pub;genObstacleMsgFromPolygon
    ros::NodeHandle private_nh, nh;
    geometry_msgs::Polygon section_poly;
    double e_car_poly_gap;
    bool emergency_car_appeared = false;
    const double AMB_escape_distance = 14.0;
    const double AMB_start_distance = 10.0;
    ros::Time amb_detected_time;
    bool makeFakebox;

};