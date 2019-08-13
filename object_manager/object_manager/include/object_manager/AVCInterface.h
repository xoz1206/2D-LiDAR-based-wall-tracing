#pragma once

#include <ros/ros.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <object_manager_msgs/combined.h>
#include <visualization_msgs/MarkerArray.h>
#include <map>
#include <mutex>
#include <std_msgs/Bool.h>
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include "object_manager/genAutowareObstacles.h"
#include <geometry_msgs/TwistStamped.h>

class AVCInterface{
    // 생성자를 통해 여러 publisher 초기화 
    AVCInterface(): private_nh("~"){//singletone. AVCInterface object must be one
        estop_pub = nh.advertise<std_msgs::Bool>("mission_estop", 100);
        obstacles_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("tracked_objects", 100);
        goal_speed_pub = nh.advertise<std_msgs::Float32>("mission_manager_maxvel", 10);
        debug_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles_debug", 100);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd2", 10);

        twist_sub = nh.subscribe("twist_cmd", 10, &AVCInterface::twistCB, this);

        // 두개의 멤버함수 호출.
        loadPredefinedObstacles();// 정의된 장애물에 대한 정보를 로딩하는 함수.
        initializeEstopAndObstacles(); // 여러가지 초기화 작업과 정의될 장애물에 대한 정보를 셋팅하는 함수.
    }
public:
    // 클래스 자체를 반환하는 함수.
    static AVCInterface* getInstancePtr(){
        //static AVCInterface* self;
        if (!self) self = new AVCInterface();
        return self;
    }
    //Estop 설정하는 함수.
    void enableEstop(bool value){
        processEstop.data = value;
    }
    // 인자로는 이름과 장애물 객체가 넘어온다.
    void setObstacle(const std::string key, const autoware_msgs::DetectedObject& obstacle){
        //std::map<std::string, autoware_msgs::DetectedObject> enabledObstacles;
        /*
            map 컨테이너는 key와 value를 쌍으로 저장하는 컨테이너이다.
            key는 컨테이너에 중복 저장될 수 없으며, 중복 key를 저장해야 한다면 multimap을 사용해야한다.
            빠른 원소찾기와 균형 이진트리를 이용한 로그시간 검색 복잡도를 보장. 
            [] 연산자를 사용해 원소추가 가능, insert() 멤버함수로도 가능.
            pair 객체가 들어가는 것이므로, 접근할 때 key는 v.first, value는 v.second로 접근한다.
         */
        enabledObstacles[key] = obstacle;    //overwriting is included , 특별한 선언없이 바로 삽입 가능.
        obstacles_msg.objects.resize(0); // obstacles_msg 초기화
        // 장애물에 해당하는 정보를 obstacles_msg.objects에 저장한다.
        // enabledObstacles 객체는 계속 장애물을 누적해가고, obstacles_msg.objects vector는 계속 초기화 한다. 
        // 따라서 계속 누적된 장애물이 입력된다.
        for(auto p : enabledObstacles)
            obstacles_msg.objects.push_back(p.second);
    }
    // 장애물을 지우는 함수.
    void eraseObstacle(const std::string key){
        enabledObstacles.erase(key); // 자료형이 map인 enabledObstacles객체에서 key와 그에 해당하는 value를 제거한다.
        obstacles_msg.objects.resize(0); // obstacles_msg.objects 객체를 초기화 한다.
        for(auto p : enabledObstacles) // 장애물을 지운 나머지 장애물을 메시지에 보낸다.
            obstacles_msg.objects.push_back(p.second);
    }
    // 속도를 msg에 담아 publish함수.
    void publishGoalSpeed(double goal){
        std_msgs::Float32 msg;
        msg.data = goal;
        goal_speed_pub.publish(msg);
    }
    // estop에 대한 정보와 장애물에 대한 정보를 최종적으로 publish한다. 
    void publish(){
        estop_pub.publish(processEstop);
        obstacles_pub.publish(obstacles_msg);

        publishDebugObstaclesMarkers();//erase this in avc contest
    }
    // 생성자에서 처음 한번 호출되는 함수.
    void initializeEstopAndObstacles(){
        enableEstop(false);
        /*
            void enableEstop(bool value){
                processEstop.data = value;
            }
            
            processEstop 선언부
            std_msgs::Bool processEstop;
            
            std_msgs::Bool 메시지 내부
                bool data
        */
        enabledObstacles = {};// enabledObstacles 초기화, 자료형 map
        obstacles_msg.objects.resize(0); // obstacles_msg.objects 객체 초기화.
        setPredefinedObstacles();  //미리 정의될 장애물을 설정하는 함수.
    }
    //
    void publishDebugObstaclesMarkers(){
        //visualization_msgs::MarkerArray 메시지 내부. Marker[] markers
        /*
            uint8 ARROW=0
            uint8 CUBE=1
            uint8 SPHERE=2
            uint8 CYLINDER=3
            uint8 LINE_STRIP=4
            uint8 LINE_LIST=5
            uint8 CUBE_LIST=6
            uint8 SPHERE_LIST=7
            uint8 POINTS=8
            uint8 TEXT_VIEW_FACING=9
            uint8 MESH_RESOURCE=10
            uint8 TRIANGLE_LIST=11
            uint8 ADD=0
            uint8 MODIFY=0
            uint8 DELETE=2
            uint8 DELETEALL=3
            std_msgs/Header header
            string ns
            int32 id
            int32 type
            int32 action
            geometry_msgs/Pose pose
            geometry_msgs/Vector3 scale
            std_msgs/ColorRGBA color
            duration lifetime
            bool frame_locked
            geometry_msgs/Point[] points
            std_msgs/ColorRGBA[] colors
            string text
            string mesh_resource
            bool mesh_use_embedded_materials
         */
        visualization_msgs::MarkerArray m;
        for(auto& p : enabledObstacles)
            //genAutowareObstacles.cpp에 있는 함수 호출. 어떻게 표시를 할 것인가가 정의된다.    
            addDebugMarkersFromObstacles(m, p.second);
        // 최종적으로 어디에 어떤 그림으로 몇초동안 표시할 것인가 를 정한 객체를 publish 한다.
        debug_pub.publish(m);
    }
    // curpos 현재 pose정보를 받아오는 함수.
    void setCurpos(const geometry_msgs::PoseStamped& curpos){
        /*
            std_msgs/Header header
            geometry_msgs/Pose pose
         */
        this->curpos = curpos;
    }
    // curpos를 읽어올 수만 있는 함수. 변경 불가능
    const geometry_msgs::PoseStamped& getCurpos() const{
        return curpos;
    }
    // mutex lock을 걸어주는 함수
    void lock(){
        avc_lock.lock();
    }
    // mutex lock을 풀어주는 함수
    void unlock(){
        avc_lock.unlock();
    }
    // 직진인지 회전인지 확인하고 그에 대한 twist 정보를 publish 하는 함수.
    void twistCB(const geometry_msgs::TwistStampedConstPtr& ptr){
        /*
            geometry_msgs::TwistStamped 메시지 내부
                std_msgs/Header header
                geometry_msgs/Twist twist

            geometry_msgs/Twist 메시지 내부
                # This expresses velocity in free space broken into its linear and angular parts.
                Vector3  linear         // x, y, z
                Vector3  angular        // x, y, z
         */
        geometry_msgs::TwistStamped twist = *ptr;
        //회전이 가능하다고 설정이 되어있다면,
        if (bControlVelocity){            
            bool isLinearTwist = true; // 기본적으로는 직진.
            if (std::fabs(ptr->twist.angular.z) > 0.01) isLinearTwist = false; // 만약 꺽여야한다면 아니라는 뜻으로 false로 바꿔준다.
            if (isLinearTwist) twist.twist.linear.x = twist_v; // 만약 직선이라면 setGoalVelocity()함수를 통해 설정한 twist_v 값을 대입한다.
        }
        // twist 정보를 publish 한다.
        twist_pub.publish(twist);
    }
    // 직진속도를 설정하는 함수.
    void setGoalVelocity(double v){
        twist_v = v;
    }

    //회전을 가능하게 해주는 함수.
    void enableControlTwist(bool value){
        bControlVelocity = value;
    }
private:
    //미리 정의된 장애물을 로딩하는 함수., 생성자에서 처음 한번 호출된다.
    void loadPredefinedObstacles(){
        constexpr static int NEWLINE_CHECKER = -99999999;
        std::vector<double> poly_raw;
        //미리 정의된 장애물의 polygons을 가져온다.
        /*
            #obstacles debug member
            predefined_polygons: [-467.253, -15.0654, -400.819, -15.1934, -99999999
                ,-503.198, -9.70501, -439.557, -5.7491, -99999999
                ,-355.639, 4.19814, -60.2589, 3.84352, -99999999]
         */
        if (!private_nh.getParam("predefined_polygons", poly_raw)) //
            throw std::runtime_error("set debug_polygons!");
        
        std::vector<geometry_msgs::Polygon> polys;
        /* geometry_msgs::Polygon 내부
            #A specification of a polygon where the first and last points are assumed to be connected
            Point32[] points
         */
        polys.emplace_back(); // 먼저 공백을 삽입하고 마지막 원소의 값으로 넣어준다.
        for(auto it = poly_raw.begin(); it != poly_raw.end(); ){
            geometry_msgs::Point32 p;
            p.x = *it++;
            p.y = *it++;
            polys.back().points.push_back(p); // 마지막원소 참조후 push_back()
            if (*it == NEWLINE_CHECKER) {
                polys.emplace_back(); //다음 point 객체가 들어갈 공간 생성.
                it++;
            }
        }
        polys.pop_back(); // ploys객체의 마지막은 emplace_back을 통해 빈 방이 생성되므로 하나 삭제해준다.
        
        obstacles_debug.header.frame_id = "map"; // 오토웨어 메시지의 header.frame_id 정의
        obstacles_debug.header.stamp = ros::Time::now(); // header.stamp 현재 시간으로 변경.
        for(auto& poly : polys){
            obstacles_debug.objects.emplace_back(); //오토웨어 메시지의 object vector의 한 칸을 생성한다. 
            genObstacleMsgFromPolygon(obstacles_debug.objects.back(), poly, 0.5); //genAutowareObstacles.cpp파일에 있는 함수 호출.
        }
    }
    // 미리 정의된 장애물을 설정하는 함수.
    void setPredefinedObstacles(){
        for(size_t i = 0 ; i < obstacles_debug.objects.size(); ++i){
            std::string n = "debug_" + std::to_string(i);
            setObstacle(n, obstacles_debug.objects[i]);
        }
    }


private:
    ros::Publisher estop_pub, obstacles_pub, goal_speed_pub, twist_pub;
    ros::Subscriber twist_sub;
    ros::NodeHandle nh, private_nh;

    std::mutex avc_lock;
    std_msgs::Bool processEstop;
    std::map<std::string, autoware_msgs::DetectedObject> enabledObstacles;

    autoware_msgs::DetectedObjectArray obstacles_msg;    

    ros::Publisher debug_pub;
    static AVCInterface* self;

    autoware_msgs::DetectedObjectArray obstacles_debug;

    geometry_msgs::PoseStamped curpos;

    bool bControlVelocity;
    double twist_v, twist_w;
};
