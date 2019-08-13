#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <mutex>
#include <dynamic_reconfigure/server.h>
#include <object_manager/ObjectManagerConfig.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PolygonStamped.h>
#include "object_manager/mission_handler.h"
#include "object_manager/ROIcheck.h"
#include "object_manager_msgs/combined.h"
#include <object_manager/AVCInterface.h>

#define AVC_CONTROL_RATE 100
/*
    This node subscribes to "DetectedObjectArray" and "BoundingBoxArray" topic from 
    each node. They must be dual, so publisher must publish 
    "DetectedObjectArray" and "BoundingBoxArray" together at the same time with same size
*/

/* All resources are released after publishing */

class ObjectManager{
public:
    // 구조체 두개 정의
    struct ObjectInfoType{
        autoware_msgs::DetectedObjectArray detectedObjAry;
        jsk_recognition_msgs::BoundingBoxArray boundingBoxAry;
        // private 부분에 정의 적어둠.
    };
    struct MissionInfo{
        int mission_identifier;
        geometry_msgs::PolygonStamped poly;
    };
public:
    // 생성자. 노드핸들러 초기화 및 estop 변수 초기화.
    ObjectManager(): private_nh("~"), estop(false){
        std::vector<std::string> combined_obj_ary_topic_names;
        //get params
        if(!private_nh.getParam("combined_obj_ary_topic_names", combined_obj_ary_topic_names)) // string vector
            throw std::runtime_error("set combined_obj_ary_topic_names!");
        if(!private_nh.getParam("vision_idx", vision_idx)) // int
            throw std::runtime_error("set vision_idx!");
        if(!private_nh.getParam("spat_idx", spat_idx)) // int
            throw std::runtime_error("set spat_idx!");
        if(!private_nh.getParam("AMB_idx", AMB_idx)) // int
            throw std::runtime_error("set AMB_idx!");
        
        // 미션 이름의 갯수만큼 크기를 정해준다.
        objInfoVec.resize(combined_obj_ary_topic_names.size());
        use_flags.resize(combined_obj_ary_topic_names.size());

        //declare detected obj subscribers
        // 각각의 이름에 하나씩 subscriber 객체를 선언해준다. 그리고 그 하나하나를 subVec에 저장한다.
        for(size_t i = 0 ; i < combined_obj_ary_topic_names.size(); ++i) {
            subVec.push_back(
                nh.subscribe<object_manager_msgs::combined>(
                    combined_obj_ary_topic_names[i],
                    10, 
                    std::bind(&ObjectManager::combinedSub, this, i , std::placeholders::_1)
                    //첫번째 인자 : 호출할 함수, 두번째 인자 : 호출될 함수의 첫번째 인자, 세번째 인자 : 호출될 함수의 두번째 인자(변화가능))
                )
            );
        }

        // topic 이름들 출력,,
        ROS_INFO("subscribing topics - ");
        for(size_t i = 0 ; i < combined_obj_ary_topic_names.size(); ++i)
            ROS_INFO("    %s", combined_obj_ary_topic_names[i].c_str());
        // publisher, subscriber 정의.
        detected_object_ary_pub = nh.advertise<autoware_msgs::DetectedObjectArray>("fused_detected_objects", 10);
        bounding_box_ary_pub = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("fused_bounding_boxes", 10);
        cur_mission_poly_pub = 
            nh.advertise<geometry_msgs::PolygonStamped>("cur_mission_poly", 10);
        curposSub = nh.subscribe("ndt_pose", 10, &ObjectManager::curposCB, this);
        
        //dynamic reconfigure settings ,실행중인 노드의 외부에서 command나 GUI를 통해 값을 변경한다.
        // 자세한 내용은 private 부분에 적혀있다.
        dyn_param_cb = boost::bind(&ObjectManager::param_callback, this, _1, _2);
        dyn_param_server.setCallback(dyn_param_cb);

        //load mission polygons
        std::vector<std::string> polygon_names;
        std::vector<geometry_msgs::PolygonStamped> polygons;
        // polygon_names: ["mission_none_poly", "pedestrian_poly", "traffic_sign_poly", "accident_vehicle_poly", "emergency_vehicle_poly"]
        // 미션이름이 담긴 vector를 가져온다.
        if (!private_nh.getParam("polygon_names", polygon_names))
            throw std::runtime_error("set polygon_names!");
        for(auto& name : polygon_names){
            std::vector<double> poly_tmp;
            /*
                pedestrian_poly: [159.697, -196.695, 185.227, -160.318, 297.77, -222.071, 280.162, -257.166]
                traffic_sign_poly: [-321.8, -160.957, 24.3253, -181.854, 75.9049, -60.5549, -43.1654, -36.4437, -307.542, -32.4161]
                accident_vehicle_poly: [-512.823, -43.541, -410.975, -43.8235, -410.912, 22.6315, -503.897, 24.1338]
                emergency_vehicle_poly: [-403.32, -40.8276, 27.2968, -26.5862, 25.3314, 21.9943, -398.567, 26.411]
             */
            // 미션이름에 맞는 poly vector를 가져온다.
            if (!private_nh.getParam(name.c_str(), poly_tmp))
                throw std::runtime_error("polygon - set " + name);
            else {
                /* geometry_msgs::PolygonStamped 메시지 내부.
                    std_msgs/Header header
                    geometry_msgs/Polygon polygon

                    geometry_msgs/Point32[] points

                    float32 x
                    float32 y
                    float32 z
                 */
                polygons.emplace_back(); // 새로운 공간 생성.
                for(auto it = poly_tmp.begin(); it != poly_tmp.end();){
                    polygons.back().polygon.points.emplace_back(); // 새로 생긴 공간의 point가 들어갈 공간 생성.
                    polygons.back().polygon.points.back().x = *it++;
                    polygons.back().polygon.points.back().y = *it++;
                }
            }
        }
        // 이 코드는 수행하지 않게 해두었다. false이기때문.
        bool show_poly = false;
        private_nh.getParam("show_poly", show_poly); // yaml 파일에도 false로 되어있다.
        if (show_poly){
            for (auto& poly : polygons){
                ros::Rate(1).sleep();//needgenObstacleMsgFromPolygonviz
                poly.header.frame_id = "mapgenObstacleMsgFromPolygon"
                poly.header.stamp = ros::TigenObstacleMsgFromPolygon
                poly.header.seq++;
                //cur_mission_poly_pub.publisgenObstacleMsgFromPolygone header field will not be used in the future
            }
            ros::Rate(1).sleep();//need to genObstacleMsgFromPolygon
        }

        //map polygon and identifier. This genObstacleMsgFromPolygont on config file.
        mission_info_vec.resize(N_MISSION);genObstacleMsgFromPolygon // 미션의 수만큼 vector의 크기를 resize 한다.
        mission_info_vec[MISSION_NONE].mission_identifier = MISSION_NONE; // 미션의 이름 정의
        mission_info_vec[MISSION_NONE].poly = polygons[MISSION_NONE];// vector polygons의 MISSION_NONE에 해당하는 polygons 정보를 저장한다.
        mission_info_vec[MISSION_PEDESTRIAN].mission_identifier = MISSION_PEDESTRIAN;
        mission_info_vec[MISSION_PEDESTRIAN].poly = polygons[MISSION_PEDESTRIAN];
        mission_info_vec[MISSION_TRAFFIC_SIGN].mission_identifier = MISSION_TRAFFIC_SIGN;
        mission_info_vec[MISSION_TRAFFIC_SIGN].poly = polygons[MISSION_TRAFFIC_SIGN];
        mission_info_vec[MISSION_ACCIDENT_VEHICLE].mission_identifier = MISSION_ACCIDENT_VEHICLE;
        mission_info_vec[MISSION_ACCIDENT_VEHICLE].poly = polygons[MISSION_ACCIDENT_VEHICLE];
        mission_info_vec[MISSION_EMERGENCY_VEHICLE].mission_identifier = MISSION_EMERGENCY_VEHICLE;
        mission_info_vec[MISSION_EMERGENCY_VEHICLE].poly = polygons[MISSION_EMERGENCY_VEHICLE];
        
        // 자료형이 map인 컨테이너들 (key, value) 실제로는 <int, int>
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_NONE]                 = 0;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_PEDESTRIAN]           = 1;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_TRAFFIC_SIGN]         = 3;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_ACCIDENT_VEHICLE]     = 4;
        MISSION_IDENTIFIER_TO_NUMBER[MISSION_EMERGENCY_VEHICLE]    = 5;

        // 처음 시작하는 미션을 설정해줄 수 있는 부분. 처음 실행하는 미션은 사고차량 미션이다.
        cur_mission = mission_info_vec[MISSION_ACCIDENT_VEHICLE];
        ROS_WARN("current mission : %d", 
            MISSION_IDENTIFIER_TO_NUMBER[cur_mission.mission_identifier]);

        //start avc interface 
        AVCInterface::getInstancePtr();

        //ok code
        ndt_ok = false;
        sensing_ok = false;
        //클래스 멤버함수를 쓰레드로 실행시키는 방법 , 람다를 사용했다. 
        std::thread([&](){
            // 쓰레드로 수행된다.
            ros::Publisher ok_pub = nh.advertise<std_msgs::Bool>("object_manager_ok", 10);
            ros::Rate loop_rate(10);
            std_msgs::Bool ok;
            while(ros::ok()){
                ok.data = ndt_ok & sensing_ok; // 둘다 참이여야지 참이 나온다. 
                ok_pub.publish(ok);
		        if (ok.data)	break; // 둘다 ok가 될 때까지 계속 수행. // 둘다 ok가 되면 모든 설정이 끝난 것.
                loop_rate.sleep();
            }
        }).detach();// detach를 사용함으로써 main()함수와 병렬적으로 수행시킨다. 
    }

    // 쓰레드 종료 조건중 하나.
    void combinedSub(int identifier, const object_manager_msgs::combinedConstPtr& ptr){
        sensing_ok = true;
        //critical section mutex lock으로 보호
        AVCInterface::getInstancePtr()->lock();
        // 현재 mission을 수행하는 함수.
        mission_handler.run(
            *AVCInterface::getInstancePtr(),combined
            cur_mission.mission_identifier, combined
            *ptr
        );
        AVCInterface::getInstancePtr()->unlock();

        //objInfoVec[identifier] = *ptr;
        //out_bounding_boxes_.header = ptr->header;
        //publishDebuggingMsgs();
    }
    // object에 대한 정보를 publish하는 함수.
    void object_publish(){
        // 초기화 작업.
        out_bounding_boxes_.boxes.clear();
        out_bounding_boxes_.boxes.resize(0);
        // 미션의 갯수만큼 크기가 정해진 objInfoVec객체 
        for(size_t i = 0; i < objInfoVec.size(); ++i){
            if(!use_flags[i]) continue; // flag가 0이면 다음 작업 수행 X
            // 위에 주석처리된 부분이 objInfoVec vector를 정의하는 부분. 주석처리때문에 수행에 문제있음
            object_manager_msgs::combined& objInfo = objInfoVec[i]; 
            for(size_t j = 0; j < objInfo.bb_ary.boxes.size(); ++j){
                out_bounding_boxes_.boxes.push_back(objInfo.bb_ary.boxes[j]);
            }
        }
        // bounding box에 대한 정보 publish
        bounding_box_ary_pub.publish(out_bounding_boxes_);        
    }

    // 실행중인 노드의 외부에서 command나 GUI를 통해 값을 변경할때 사용되는 callback 함수.
    void param_callback(object_manager::ObjectManagerConfig &config, uint32_t level){
        use_flags[vision_idx] = config.vision_flg;
        use_flags[spat_idx] = config.spat_flg;
        use_flags[AMB_idx] = config.AMB_flg;
        // mission 의 config 를 바꿔준다.
        mission_handler.set_param(config);        
    }

    // 현재 위치 callback 함수. 쓰레드 종료조건 중 하나.
    void curposCB(const geometry_msgs::PoseStampedConstPtr& ptr){
        //초기화 작업.
        ndt_ok = true;
        static int seq = 0;
        curpos = *ptr;
        bool cur_mission_found = false;
        //find cur mission area. If mission changed, initialize estop and objects states.
        //critical section, mutex lock 사용.
        AVCInterface::getInstancePtr()->lock();
        AVCInterface::getInstancePtr()->setCurpos(curpos);
        AVCInterface::getInstancePtr()->publish();
        AVCInterface::getInstancePtr()->unlock();

        for(auto& mission_info : mission_info_vec){
            // ROIcheck.cpp파일에 있는 함수.
            if (true == isPointInPolygon(curpos.pose.position.x, curpos.pose.position.y, mission_info.poly.polygon))
            {
                cur_mission_found = true; // 현재 미션이 무엇인지 찾았음.
                if (cur_mission.mission_identifier != mission_info.mission_identifier){
                    cur_mission = mission_info;
                    AVCInterface::getInstancePtr()->lock();
                    // enabledObstacles. obstacles_msg.objects 객체 초기화 및 장애물 setting하는 함수.
                    AVCInterface::getInstancePtr()->initializeEstopAndObstacles(); 
                    AVCInterface::getInstancePtr()->unlock();
                    ROS_WARN("[MISSION %d] Handler on!", 
                        MISSION_IDENTIFIER_TO_NUMBER[cur_mission.mission_identifier]);
                    break;
                }
            }
        }
        // 현재 미션이 0이라면 MISSION_NONE을 넣어준다.
        if (!cur_mission_found) cur_mission = mission_info_vec[MISSION_NONE];

        //publish debug polygon
        cur_mission.poly.header.stamp = ros::Time::now();
        cur_mission.poly.header.frame_id = "map";
        cur_mission.poly.header.seq = seq++;
        cur_mission_poly_pub.publish(cur_mission.poly);
    }

    void Run(){
        // subscribe 함수 호출
        ros::spin();
    }
    //디버깅 메시지를 publish 해서 확인하려고 할때 사용하는 함수.
    void publishDebuggingMsgs(){
        object_publish();   
    }

private:
    //estop dependent
    ros::Publisher estop_pub;
    bool estop;

    //autoware obstacles dependent
    ros::Publisher autoware_obstacle_pub;
    autoware_msgs::DetectedObjectArray autoware_obstacles;  // 메시지 객체 선언  

    //fundamental members
    ros::NodeHandle nh, private_nh; // 두개의 노드핸들러 선언
    std::vector<object_manager_msgs::combined> objInfoVec; 
    /* object_manager_msgs의 정보를 담는 vector 자료형 objInfoVec 선언.
        std_msgs/Header header
        jsk_recognition_msgs/BoundingBoxArray bb_ary
        autoware_msgs/DetectedObjectArray obj_ary
    */
    std::vector<ros::Subscriber> subVec;
    /*
        Subscriber를 담는 vector 선언
        즉, subscriber 객체들이 저장될 vector 선언.
     */
    std::vector<bool> use_flags;
    // flag에 대한 정보를 위한 bool 형 vector 선언
    int vision_idx, spat_idx, AMB_idx;

    ros::Publisher detected_object_ary_pub;
    ros::Publisher bounding_box_ary_pub;
    ros::Subscriber curposSub;
    // 사용자가 정의한 두 개의 publisher와 한 개의 subscriber
    
    jsk_recognition_msgs::BoundingBoxArray out_bounding_boxes_;
    //정의된 메시지 객체 선언.
    /* jsk_recognition_msgs::BoundingBoxArray 내부
        # BoundingBox represents a oriented bounding box.
        Header header
        geometry_msgs/Pose pose
        geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
        # You can use this field to hold value such as likelihood
        float32 value
        uint32 label
     */
    dynamic_reconfigure::Server<object_manager::ObjectManagerConfig> dyn_param_server;
    dynamic_reconfigure::Server<object_manager::ObjectManagerConfig>::CallbackType dyn_param_cb;
    /*
        dynamic_reconfigure 패키지는 어떤 로봇의 이동 속도나 센서 설정 등을 변경할 때마다 로봇을 재시작해야 한다는 단점을 해결하기 위해 
        나온 패키지로 GUI(rqt) 또는 command를 통해 gain값을 외부에서 즉시 수정할 수 있도록 해준다.
        dynamic_reconfigure server를 정의하고 callback 함수를 호출하는 객체를 만들어 server에 전달.
        server가 동작하는 동안 node가 외부 파라미터 변동에 대한 요청을 받을 수 있도록 한다. 
        변동 요청을 받으면 callback함수를 호출하여 수행시킨다.
    */
    //mission dependent
    MissionHandler mission_handler; // MissionHandler 클래스에 주석 달아놓음. , 미션에 맞춰서 해야하는 행동들 정의.
    geometry_msgs::PoseStamped curpos;
    /*
        # A Pose with reference coordinate frame and timestamp
        Header header
        Pose pose
     */
    std::vector<MissionInfo> mission_info_vec;
    /* MissionInfo 구조체
        struct MissionInfo{
        int mission_identifier;
        geometry_msgs::PolygonStamped poly;
    };
     */
    MissionInfo cur_mission;
    ros::Publisher cur_mission_poly_pub;
    
    AVCInterface* avc_interface_ptr; // 객체 생성.
    std::map<int, int> MISSION_IDENTIFIER_TO_NUMBER; // 자료형이 map인 객체 생성,, 

    //ok members
    bool ndt_ok, sensing_ok;
    ros::Publisher ok_pub;
};

//main 함수 
int main(int argc, char *argv[]){
    ros::init(argc, argv, "object_manager");
    //object_manger 생성자 호출
    ObjectManager object_manager;
    //object_mager의 멤버 함수 Run() 실행
    object_manager.Run();

    return 0;
}
