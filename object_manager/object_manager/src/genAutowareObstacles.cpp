#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Polygon.h>
#include "object_manager/genAutowareObstacles.h"
#include "object_manager/ROIcheck.h"

// bounding box정보를 받아와 장애물을 생성하는 함수.
void genObstacleMsgFromBoundingbox(autoware_msgs::DetectedObject& output,
    const jsk_recognition_msgs::BoundingBox& input){
    constexpr static int N_POINT_ALONG_LINE = 10000;        

    output.header.stamp = ros::Time::now();
    output.id = 100001;
    output.pose.position = input.pose.position;
    output.pose.position.z = 1.12;
    output.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    output.dimensions = input.dimensions;
    output.indicator_state = 0;
    output.behavior_state = 0;
    output.velocity_reliable = false;

    //make polygon with 20 points
    /*
        * * * * * *
        *         *
        *         *
        *         *
        *         *
        * * * * * *
    */
    //I'll make points of rectangle's round that is orthogonal to axis first, and
    //then rotate the points.
    // 높이가 2.5인 장애물 생성.
    float l_x = input.dimensions.x;
    float l_y = input.dimensions.y;
    float center_x = input.pose.position.x;
    float center_y = input.pose.position.y;
    float center_z = 2.5;
    auto& poly_points = output.convex_hull.polygon.points;
    poly_points.resize(N_POINT_ALONG_LINE * 4 - 4);
    int polygon_idx = 0;

    //bottom left to bottom right
    poly_points[polygon_idx].x = center_x - l_x/2.0f;
    poly_points[polygon_idx].y = center_y - l_y/2.0f;
    poly_points[polygon_idx].z = center_z;
    polygon_idx++;
    for(int i = 1 ; i < N_POINT_ALONG_LINE - 1; ++i){
        poly_points[polygon_idx].x = center_x - l_x/2.0f + l_x / N_POINT_ALONG_LINE * i;
        poly_points[polygon_idx].y = center_y - l_y/2.0f;
        poly_points[polygon_idx].z = center_z;
        polygon_idx++;
    }
    //bottom right to upper right
    poly_points[polygon_idx].x = center_x + l_x/2.0f;
    poly_points[polygon_idx].y = center_y - l_y/2.0f;
    poly_points[polygon_idx].z = center_z;
    polygon_idx++;
    for(int i = 1 ; i < N_POINT_ALONG_LINE - 1; ++i){
        poly_points[polygon_idx].x = center_x + l_x/2.0f;
        poly_points[polygon_idx].y = center_y - l_y/2.0f + l_y / N_POINT_ALONG_LINE * i;
        poly_points[polygon_idx].z = center_z;
        polygon_idx++;
    }
    //upper right to upper left
    poly_points[polygon_idx].x = center_x + l_x/2.0f;
    poly_points[polygon_idx].y = center_y + l_y/2.0f;
    poly_points[polygon_idx].z = center_z;
    polygon_idx++;
    for(int i = 1 ; i < N_POINT_ALONG_LINE - 1; ++i){
        poly_points[polygon_idx].x = center_x + l_x/2.0f - l_x / N_POINT_ALONG_LINE * i;
        poly_points[polygon_idx].y = center_y + l_y/2.0f;
        poly_points[polygon_idx].z = center_z;
        polygon_idx++;
    }
    //upper left to bottom left
    poly_points[polygon_idx].x = center_x - l_x/2.0f;
    poly_points[polygon_idx].y = center_y + l_y/2.0f;
    poly_points[polygon_idx].z = center_z;
    polygon_idx++;
    for(int i = 1 ; i < N_POINT_ALONG_LINE - 1; ++i){
        poly_points[polygon_idx].x = center_x - l_x/2.0f;
        poly_points[polygon_idx].y = center_y + l_y/2.0f - l_y / N_POINT_ALONG_LINE * i;
        poly_points[polygon_idx].z = center_z;
        polygon_idx++;
    }
        
    //rotate points
    // 회전된 x, y 좌표를 얻는다. 
    double rot_angle = tf::getYaw(input.pose.orientation);
    double s = std::sin(rot_angle);
    double c = std::cos(rot_angle);
    for (auto it = output.convex_hull.polygon.points.begin(); 
        it != output.convex_hull.polygon.points.end();
        ++it){
        double x_new = it->x*c - it->y*s;
        double y_new = it->x*s + it->y*c;
        it->x = x_new;
        it->y = y_new;
    }
}

// 아래 genObstacleMsgFromPolygon 함수와 비슷. 
void genObstacleMsgFromRectanglePoly(autoware_msgs::DetectedObject& output,
    const geometry_msgs::Polygon& poly, double gap_between_poly_point){

    output.id = 100001;
    //find position
    double x_center = 0;
    double y_center = 0;
    for(auto& point : poly.points){
        x_center += point.x;
        y_center += point.y;
    }
    x_center /= poly.points.size() * 1.0; y_center /= poly.points.size() * 1.0;
    
    output.header.stamp = ros::Time::now();
    output.pose.position.x = x_center;
    output.pose.position.y = y_center;
    output.pose.position.z = 1.12;
    output.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    output.dimensions.x = 300; 
    output.dimensions.y = 3.5;
    output.indicator_state = 3;
    output.behavior_state = 0;
    output.velocity_reliable = true;
    
    geometry_msgs::Polygon target_poly;
    
    for(int i = 0 ; i < 4; ++i){
        const double x_start = poly.points[i % 4].x;
        const double y_start = poly.points[i % 4].y;
        if ((std::fabs(x_start) <= 1.0) && (std::fabs(y_start) <= 1.0)) continue; //filter noise

        const double x_end = poly.points[(i+1) % 4].x;
        const double y_end = poly.points[(i+1) % 4].y;

        const double l = std::sqrt(
            std::pow(x_end - x_start, 2) +
            std::pow(y_end - y_start, 2)
        );
        const int N_POINT_ALONG_LINE = (int)(l / gap_between_poly_point) + 1;
        
        const double dx = (x_end - x_start) / N_POINT_ALONG_LINE;
        const double dy = (y_end - y_start) / N_POINT_ALONG_LINE;

        for(int j = 0; j < N_POINT_ALONG_LINE; ++j){
            geometry_msgs::Point32 p;
            p.x = x_start + dx*j;
            p.y = y_start + dy*j;
            p.z = output.pose.position.z;
            target_poly.points.push_back(p);
        }
    }
    output.convex_hull.polygon = target_poly;
}

// 새로 만들어준 맨 마지막 공간과 poly좌표, 실수가 인자로 넘어온다.
void genObstacleMsgFromPolygon(autoware_msgs::DetectedObject& output,
    const geometry_msgs::Polygon& poly, double gap_between_poly_point){
        
    if (!poly.points.size()) return; // 만일 넘어온 poly 좌표의 크기가 0이면 함수 종료.
    output.id = 100001; 
    //find position
    double x_center = 0;
    double y_center = 0;

    // 중앙 좌표를 찾기 위한 작업.
    for(auto& point : poly.points){
        x_center += point.x;
        y_center += point.y;
    }
    x_center /= poly.points.size(); y_center /= poly.points.size();

    // 장애물의 pose 정보 입력.
    output.header.stamp = ros::Time::now();
    output.pose.position.x = x_center;
    output.pose.position.y = y_center;
    output.pose.position.z = 1.12;
    output.pose.orientation = tf::createQuaternionMsgFromYaw(0);
    output.indicator_state = 3;
    output.behavior_state = 0;
    output.velocity_reliable = true;
    
    geometry_msgs::Polygon target_poly;
    
    const int N_POLYGON_POINT = (int)poly.points.size(); // poly의 좌표수 반환.
    for(int i = 0 ; i < N_POLYGON_POINT; ++i){
        // polygon의 시작 point좌표를 얻는다.
        const double x_start = poly.points[i % N_POLYGON_POINT].x; 
        const double y_start = poly.points[i % N_POLYGON_POINT].y;
        // 실수의 절대값을 구하고 만약 실수의 절대값이 반지름이 1인 원 보다 안쪽이면 filtering을 수행한다.
        if ((std::fabs(x_start) <= 1.0) && (std::fabs(y_start) <= 1.0)) continue; //filter noise
        // polygon의 끝 point좌표를 얻는다.
        const double x_end = poly.points[(i+1) % N_POLYGON_POINT].x;
        const double y_end = poly.points[(i+1) % N_POLYGON_POINT].y;

        //2차원 상에서의 polygon의 start point와 end point 사이의 거리를 측정한다. l : 두 점 사이의 거리
        const double l = std::sqrt(
            std::pow(x_end - x_start, 2) +
            std::pow(y_end - y_start, 2)
        );
        // 지금부터 시작점부터 끝점을 이은 선 위에 l / gap_between_poly_point + 1 값을 간격으로 점을 찍는다.  
        const int N_POINT_ALONG_LINE = (int)(l / gap_between_poly_point) + 1;
        const double dx = (x_end - x_start) / N_POINT_ALONG_LINE;
        const double dy = (y_end - y_start) / N_POINT_ALONG_LINE;
        // start point와 end point 사이의 점을 만들고 그 point 객체를 새로운 polygon vector에 저장한다. 
        for(int j = 0; j < N_POINT_ALONG_LINE; ++j){
            geometry_msgs::Point32 p;
            p.x = x_start + dx*j;
            p.y = y_start + dy*j;
            p.z = output.pose.position.z;
            target_poly.points.push_back(p);
        }
    }
    // 나중에 publish할 때 이 정보가 있어야 하므로 저장.
    output.convex_hull.polygon = target_poly;
}

// 인자로 marker와 장애물에 대한 정보가 넘어온다.
void addDebugMarkersFromObstacles(visualization_msgs::MarkerArray& markers,
    const autoware_msgs::DetectedObject& obj){
    int id = 0;
    if (markers.markers.size()) id = markers.markers.back().id + 1;

    //genObstacleMsgFromPolygon에서 최종적으로 얻은 convex_hull.polygon vector정보 사용.
    for(auto& point : obj.convex_hull.polygon.points){

        markers.markers.emplace_back();
        // marker 객체 선언 후, 정보 삽입.
        auto& marker = markers.markers.back();
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = id++;
        marker.lifetime = ros::Duration(0.2); // marker가 띄워져있는 시간?

        marker.type = visualization_msgs::Marker::SPHERE; // 원
        marker.action = visualization_msgs::Marker::ADD; // 그림.
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = point.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
    }
}