#pragma once
#include <object_manager/mission_processor.h>
#include <object_manager/genAutowareObstacles.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

static constexpr double ELAPSED_TIME_TOLERANCE = 0.01;//second
static constexpr double CLOST_DISTANCE_TOLERANCE = 3;//meter
struct Obstacle{
    //생성자를 이용한 초기화.
    Obstacle(ros::Time t, std::string n, geometry_msgs::Polygon poly, geometry_msgs::Pose pose):
        detected_time(t), name(n), polygon(poly), pose(pose) {}
    // 경과시간 얻어오는 함수.
    double getElapsedTime(ros::Time t){
        return (t - detected_time).toSec();
    }
    // 현재 위치와 x, y 사이의 거리를 반환하는 함수.
    double getDistance(double x, double y){
        double x_cur = pose.position.x;
        double y_cur = pose.position.y;
        return std::sqrt((x-x_cur)*(x-x_cur) + (y-y_cur)*(y-y_cur));
    }
    // 이름을 반환하는 함수.
    const std::string& getName() const{
        return name;
    }
    // polygon을 받아오는 함수.
    const geometry_msgs::Polygon getPolygon() const{
        return polygon;
    }
    ros::Time detected_time;
    std::string name;
    geometry_msgs::Polygon polygon;
    geometry_msgs::Pose pose;
};

class AccidentVehilceProcessor : public MissionProcessor{
public:
    // 생성자.
    AccidentVehilceProcessor(){
        car_id = 0;
    }
    // 미션 처리하는 함수.
    void processMission(AVCInterface& avc_interface,
        const object_manager_msgs::combined msg){
        auto &detectedObj = msg.obj_ary;
        
        //process only camera msg
        bool isCarMsg = false;

        for(auto it = detectedObj.obgenObstacleMsgFromPolygondObj.objects.end(); ++it){
            // 차량 발견.
            if(it->label == "car") {//genObstacleMsgFromPolygon ??? 왜 있는 거지
                isCarMsg = true;
                break;
            }
        }        
        // 차량을 발견할 때 까지 반복.
        if(!isCarMsg) 
            return;

        //collect car detectObject instance
        for(auto it = detectedObj.objects.begin(); it != detectedObj.objects.end(); ++it){
            //차량을 발견하면, 
            if(it->label == "car" && it->convex_hull.polygon.points.size()) {
                autoware_msgs::DetectedObject d;
                genObstacleMsgFromPolygon(d, it->convex_hull.polygon, 0.5); // 0.5 간격으로 polygon좌표를 늘리고 그 좌표를 convex_hull.polygon에 저장한다.
                d.label = it->label;
                // erase these lines after fixing camera bug
                if (!d.convex_hull.polygon.points.size()) {
                    continue;
                } 
                // erase end 
                std::string n = "car" + std::to_string(car_id);
                //find objects center position in map frame
                geometry_msgs::PointStamped p_velodyne, p_map;
                p_velodyne.header.frame_id = "velodyne";//hard coded
                //p_velodyne.header.stamp = ros::Time::now();
                // 장애물의 위치정보를 가져온다.
                p_velodyne.point.x = it->pose.position.x;
                p_velodyne.point.y = it->pose.position.y;
                try
                {
                    /*  tf::TransformListener.transformPoint 함수.
                        void TransformListener::transformPoint 	( 	const std::string &  	target_frame,
                                const geometry_msgs::PointStamped &  	stamped_in,
                                geometry_msgs::PointStamped &  	stamped_out	 
                            ) 			const
                        Transform a Stamped Point Message into the target frame and time 
                        This can throw all that lookupTransform can throw as well as tf::InvalidTransform. 
                     */
                    transform_listener.transformPoint("map", p_velodyne, p_map);
                }
                // try 안에서 예외가 발생해 throw가 수행되고 예외 객체에 대한 예외 핸들러로 catch 문이 수행된다.
                catch (tf::TransformException &ex){
                    ROS_ERROR("map-velodyne tf does not exist");
                    for(auto&& o : Obstacles_keep)
                        avc_interface.eraseObstacle(o.getName());
                    return;
                }
                // 사고차량의 map frame상에서의 pose를 저장한다.
                geometry_msgs::Pose pose;
                pose.position.x = p_map.point.x;
                pose.position.y = p_map.point.y;
                
                //keep in memory to erase later 
                // 장애물의 이름과 정보를 인자로 넘겨 장애물을 설정한다.
                avc_interface.setObstacle(n, d);
                // 자료형이 Obstacle이므로 인자로 4개의 정보가 들어가야 한다. 
                // 사고차량의 대한 vector에 값을 넣어준다.
                Obstacles_keep.emplace_back(ros::Time::now(), n, d.convex_hull.polygon, pose);
                car_id = (car_id + 1) % 10000000;//iteration
            }
        }

        //erase too far or old obstacle
        std::vector<std::string> eraseTargets;
        //현재 차량의 좌표를 가져온다. 
        double x_vehicle = avc_interface.getCurpos().pose.position.x;
        double y_vehicle = avc_interface.getCurpos().pose.position.y;
        ros::Time t = ros::Time::now();
        // 사고차량에 대한 정보를 
        for(auto it = Obstacles_keep.begin(); it != Obstacles_keep.end();){
            if ((it->getElapsedTime(t) > ELAPSED_TIME_TOLERANCE) && // 경과시간을 받아온다.
                (it->getDistance(x_vehicle, y_vehicle) > CLOST_DISTANCE_TOLERANCE)){ // 현재차량과 사고차량의 거리를 계산한다.
                eraseTargets.push_back(it->getName()); // 지울 장애물을 넣는다.
                it = Obstacles_keep.erase(it); // 벡터에서 값을 지우고 다음 iterator를 반환한다.
            }
            else it++;
        }

        for(auto&& n : eraseTargets)
            avc_interface.eraseObstacle(n); //avc_interface에서도 지운다.
        
        avc_interface.publish();
    }

    
    virtual void set_param(object_manager::ObjectManagerConfig &config)
    { 
    }
private:
    ros::NodeHandle nh;
    std::deque<Obstacle> Obstacles_keep;
    size_t car_id;
    tf::TransformListener transform_listener;
};
