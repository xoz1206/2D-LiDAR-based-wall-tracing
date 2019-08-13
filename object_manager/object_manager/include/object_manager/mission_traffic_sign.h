#pragma once
#include <object_manager/mission_processor.h>
#include <autoware_msgs/DetectedObject.h>
#include <object_manager/ROIcheck.h>
#include <geometry_msgs/PolygonStamped.h>

enum {
    SPAT_1= 0, SPAT_2, SPAT_3, N_SPAT
};
enum {
    DECISION_NONE, DECISION_GO, DECISION_REDLIGHT_GO, DECISION_REDLIGHT_WAIT
};

enum {SPAT_RED_LABEL = 0, SPAT_GREEN_LABEL = 1, SPAT_YELLOW_LABEL = 2};
constexpr static double WAIT_VELOCITY_MPS = 2;//7 kph

class TrafficSignProcessor : public MissionProcessor{
public:
    //생성자.
    TrafficSignProcessor() : private_nh("~"){
        //spat_stop_area_x: [-28, -135, -272]
        //decision_distance: 100
        // max_speed: 11 #40 kph
        // mission_end_speed: 11
        // dist_margin: 5
        // time_margin: 3
        if (!private_nh.getParam("spat_stop_area_x", spat_stop_area_x))
            throw std::runtime_error("set spat_stop_area_x!");
        if (!private_nh.getParam("decision_distance", decision_distance))
            throw std::runtime_error("set decision_distance!");
        if (!private_nh.getParam("max_speed", max_speed))
            throw std::runtime_error("set max_speed!");
        if (!private_nh.getParam("mission_end_speed", mission_end_speed))
            throw std::runtime_error("set mission_end_speed!");
        if (!private_nh.getParam("dist_margin", dist_margin))
            throw std::runtime_error("set dist_margin!");
        if (!private_nh.getParam("time_margin", time_margin))
            throw std::runtime_error("set time_margin!");
        decision_state = DECISION_NONE;
        current_spat = SPAT_1;//when mission started, the stop line's x coordinate is 
        //below car's x coordinate.

        bPassedAllStopline = false; // 모든 신호등을 지났는지 판단할때 사용하는 변수.
    }
    // 미션을 처리하는 함수.
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){

        auto& detectedObj = msg.obj_ary;
        
        //check if spat msg
        bool isSpatMsg = false;
        const std::vector<std::string> signal_names{"signal_0", "signal_1", "signal_2"};
        auto n = signal_names.cbegin(); // auto 키워드는 begin() 타입을 받을 수 없다. const_iterator가 아닌 iterator 타입이기 때문.
        // lvalue
        for(auto&& obj : detectedObj.objects){
            // 신호등이 검출 되었는지 아닌지 확인.
            if (obj.label == *n){
                isSpatMsg = true;
                break;
            }
            n++;
        }
        // 신호등이 검출되지않았으면 더이상 진행 할 필요없다.
        if (isSpatMsg == false)
            return ;

        //hard coded
        const double x_cur = avc_interface.getCurpos().pose.position.x; // 차량의 현재 위치를 받아온다.
        double dist = x_cur - spat_stop_area_x[current_spat]; // 신호등과 차량의 거리를 계산한다.
        // 차량의 거리가 13 meter 보다 크다면, 
        if (current_spat == SPAT_1){
            if (dist > 13){ // meter 
                dist = 13 + (dist - 13) / 0.9064611; // cos(24.9792)  /// ?????
            }
        }

        // 출력.
        ROS_WARN("[MISSION 3] dist : %lf", dist);
        
        double remaining_time = detectedObj.objects[current_spat].score;
    
        // 신호등 3개를 아직 지나지 않았다면.
        if (!bPassedAllStopline){ // 생성자를 통해서 false
            switch(decision_state){ // 처음은 생성자를 통해 DECISION_NONE 값이 들어온다.
            case DECISION_NONE:{
                ROS_WARN("DECISION__NONE");
                if (dist > decision_distance) {
                    goal_speed = max_speed; // 최대 속도로 주행을 시작한다.
                    break;
                }
                // 첫번째 if문과 마지막 else문에 동시에 걸리게 될 가능성은 없나???
                // 만약 초록불이라면 
                if (detectedObj.objects[current_spat].indicator_state == SPAT_GREEN_LABEL){
                    remaining_time -= 0.5; // tolerance
                    //남은 시간보다 갈수 있는 시간이 더 크다면 못지나간다. 따라서 속도를 낮춘다.
                    if (remaining_time < time_margin) { //not enough time with long distance
                        goal_speed = WAIT_VELOCITY_MPS;//10 kph
                    }

                    // 거리와 남은 시간을 이용해 달려야 하는 속도를 구한다. 
                    double enough_speed = dist / remaining_time;
                    
                    // 내야 된다는 속도가 자동차가 최대로 달릴 수 있는 속도보다 높다면, 못지나간다. 따라서 속도를 낮춘다. 
                    if (enough_speed > max_speed) { //vehicle's controllability lacks
                        goal_speed = WAIT_VELOCITY_MPS;//10 kph
                    }
                    // 만약 자동차라 낼수있는 속도라면, 최대 속도로 바꾸고 진행하라는 뜻의 DECISION_GO로 decision_state를 바꿔준다.
                    else {
                        goal_speed = max_speed;
                        decision_state = DECISION_GO;
                    }
                }
                // 만약 노란불이라면, 못지나간다. 따라서 속도를 낮춘다.
                else if (detectedObj.objects[current_spat].indicator_state == SPAT_YELLOW_LABEL){
                    goal_speed = WAIT_VELOCITY_MPS;
                }
                // 만약 빨간불이라면 
                else if (detectedObj.objects[current_spat].indicator_state == SPAT_RED_LABEL){
                    //아직 dist_margin보다 거리가 많이 남았다면, 빠르게 달려도 된다.
                    if (dist > dist_margin){
                        double enough_speed = (dist - dist_margin) / remaining_time;
                        // 내야 된다는 속도가 자동차라 최대로 달릴 수 있는 속도보다 높다면, 최대 속도로 설정한다. 
                        if (enough_speed > max_speed) enough_speed = max_speed;
                        goal_speed = enough_speed;
                        decision_state = DECISION_REDLIGHT_GO;
                    }
                    // dist_margin보다 거리가 조금 남았다면, 기다려야 한다.
                    else {
                        // 멈춤
                        avc_interface.enableEstop(true);
                        decision_state = DECISION_REDLIGHT_WAIT;
                    }
                }

                break;
            }
            case DECISION_GO:
                ROS_WARN("DECISION__GO");     
                // 신호등을 지나간다면            
                if (dist < 0) {
                    decision_state = DECISION_NONE;
                    current_spat++; // 다음 신호등을 체크한다.
                    if (current_spat > SPAT_3) bPassedAllStopline = true; // 모든 신호등을 지났다.
                } 
                break;
            case DECISION_REDLIGHT_GO:
                ROS_WARN("DECISION__REDLIGHT_GO");
                // 남은 시간보다 갈 수 있는 시간이 더 크면 최대 속도로 달려도 그 사이에 초록불로 바뀐다.                
                if (remaining_time < time_margin){
                    goal_speed = max_speed;
                    decision_state = DECISION_GO;
                } 
                else{
                    // 빨간불인데 신호등과의 거리가 가깝다면 멈춰야 한다.
                    if (dist < dist_margin){ // too much time with short distance
                        avc_interface.enableEstop(true);
                        decision_state = DECISION_REDLIGHT_WAIT;
                    }
                    // 신호등의 남은 시간보다 자동차가 갈 수 있는 시간이 더 작고, 신호등과의 거리가 충분히 떨어져있다면,  
                    else {
                        double enough_speed = (dist - dist_margin) / remaining_time;
                        if (enough_speed > max_speed) enough_speed = max_speed;
                        goal_speed = enough_speed;
                    }
                }
                break;
            // 빨간불이여서 멈춰야 한다.
            case DECISION_REDLIGHT_WAIT: //merely happen
                if (remaining_time < time_margin){
                    avc_interface.enableEstop(false);
                    goal_speed = max_speed;
                    decision_state = DECISION_GO;
                }
                break;
            // 예외 처리.
            default : ROS_ERROR("impossible situation happened"); break;
            }
        } 
        // 만약 3개의 모든 신호등을 지났다면. 설정해둔 속도로 주행한다.
        else goal_speed = mission_end_speed;
        
        avc_interface.publish();// estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. 
        avc_interface.publishGoalSpeed(goal_speed); // 차량의 속도 출력.
    }

    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
    }
private:
    ros::NodeHandle private_nh, nh;

    double decision_distance;
    double goal_speed;
    double max_speed, mission_end_speed;
    double dist_margin, time_margin;

    std::vector<double> spat_stop_area_x;
    int decision_state;
    int current_spat;
    bool bPassedAllStopline;

};
