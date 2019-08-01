#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <utility>
#include "sensor_msgs/LaserScan.h"
#include "rplidar_ros/rp_point_calculation_define.h"
#include <cmath>
#include "opencv2/opencv.hpp"
#include "rplidar_ros/final_direction.h"

using namespace cv;
using namespace std;


class degree_to_pointxy{
    public:
        degree_to_pointxy() :srcImage(512, 512, CV_8UC3, Scalar(255,255,255)), clear_Image(512, 512, CV_8UC3, Scalar(255,255,255))
        {
            sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, &degree_to_pointxy::run_callback, this);
            pub = nh.advertise<rplidar_ros::final_direction>("/final_direction", 1000);
        }
        void run_callback(const sensor_msgs::LaserScanConstPtr &ptr)
        {
            for(int i=0; i<ptr->ranges.size(); i++)
            {
                // 일정 범위 내 값 추출.
                if(range_threshold_min < ptr->ranges[i] && ptr->ranges[i] < range_threshold_max)
                {
                    p = make_pair(i, ptr->ranges[i]);
                    limited_ranges.push_back(p);
                }
            }
            transform(limited_ranges);
            perform_inspection(pointxy_vec, limited_ranges);
            pub.publish(fdmsg);

            imshow("srcImage", srcImage);
            waitKey(10);

            //데이터 삭제 및 초기화
            limited_ranges.clear();
            pointxy_vec.clear();
            srcImage = clear_Image.clone();
        }
        void transform(vector< pair<int, float> > vec)
        {
            float radian;
            int degree;
            float distance;

            for(int k = 0; k< vec.size(); k++)
            {
                degree = vec[k].first;
                distance = vec[k].second;

                if(0 < degree && degree <= 90) // 4 사분면
                {
                    radian = (90-degree)*CV_PI/180.0;
                    p_xy = make_pair(distance*cos(radian), (-1)*distance*sin(radian));
                    pointxy_vec.push_back(p_xy);
                }
                else if (90 < degree && degree <= 180) // 1 사분면
                {
                    radian = (degree - 90)*CV_PI/180.0;
                    p_xy = make_pair(distance*cos(radian), distance*sin(radian));
                    pointxy_vec.push_back(p_xy);
                }
                else if (180 < degree && degree <= 270) // 2 사분면
                {
                    radian = (270-degree)*CV_PI/180.0;
                    p_xy = make_pair((-1)*distance*cos(radian), distance*sin(radian));
                    pointxy_vec.push_back(p_xy);
                }
                else if (270 < degree && degree <= 360) // 3 사분면
                {
                    radian = (degree - 270)*CV_PI/180.0;
                    p_xy = make_pair((-1)*distance*cos(radian), (-1)*distance*sin(radian));
                    pointxy_vec.push_back(p_xy);
                }
            }
            print_point_using_opencv(pointxy_vec);
        }

        // 좌표 출력하는 함수

        void print_point_using_opencv(vector< pair<float, float> > vec)
        {
            for(int k = 0; k< vec.size(); k++)
            {
                // opencv 이미지 상에서 중심좌표는 250, 250 // 좌표 디버깅용
                circle(srcImage, Point(int(vec[k].first*100 + 250), int(vec[k].second*100 + 250)), 1, Scalar(255,0,0), -1);
                //ROS_INFO("x = %f , y = %f", vec[k].first, vec[k].second);

            }
                ROS_INFO("-----------------");
        }

        //four inspection perform
        void perform_inspection(vector< pair<float, float> > vec_pointxy, vector< pair<int, float> > vec_degree)
        {
            int right_check = 0;
            int left_check = 0;
            int up_check = 0;
            int sum = 0;

            pair<float, float> p;

            // 4가지 동작.
            fdmsg.go_straight = false;
            fdmsg.turn_right = false;
            fdmsg.turn_left = false;
            fdmsg.turn_back = false;
            fdmsg.go_x = false;
            fdmsg.go_y = false;

            right_check = check_right_area(vec_degree);
            left_check = check_left_area(vec_degree);
            up_check = check_up_area(vec_degree);
            
            sum = right_check + left_check + up_check;

            switch(sum)
            {
                case zero_:
                    ROS_INFO("nothing inspection");
                    break;
                case right_:
                    ROS_INFO("only right inspection");
                    p = right_area_inspection(vec_pointxy, vec_degree);
                    break;
                case left_:
                    ROS_INFO("only left inspection");
                    p = left_area_inspection(vec_pointxy, vec_degree);
                    break;  
                case up_:
                    ROS_INFO("only up inspection, turn left 90");
                    fdmsg.turn_left = true;
                    break;
                case right_left:
                    ROS_INFO("right , left inspection");
                    p = right_area_inspection(vec_pointxy, vec_degree);
                    break;
                case right_up:
                    ROS_INFO("right , up inspection");
                    p = right_up_area_inspection(vec_pointxy, vec_degree);
                    break;
                case right_left_up:            
                    ROS_INFO("right , left , up inspection, turn back 180");
                    fdmsg.turn_back = true;
                    break;
                case left_up:
                    ROS_INFO("left , up inspection");
                    p = left_up_area_inspection(vec_pointxy, vec_degree);
                    break;
            }
            if(p.first != 0 && p.second != 0)
            {
                fdmsg.go_x = p.first;
                fdmsg.go_y = p.second;
            }
            else
            {
                ROS_INFO("갈곳을 잃었습니다. ㅠㅠ");
            }
        }

        int check_right_area(vector< pair<int, float> > vec_degree)
        {
            for(int k = 0; k<vec_degree.size(); k++)
            {
                if(vec_degree[k].first > right_inspection_min && vec_degree[k].first < right_inspection_max)
                {
                    return 1;
                }
            }
            return 0;
        }
        int check_left_area(vector< pair<int, float> > vec_degree)
        {
            for(int k = 0; k<vec_degree.size(); k++)
            {
                if(vec_degree[k].first > left_inspection_min && vec_degree[k].first < left_inspection_max)
                {
                    return 10;
                }
            }
            return 0;
        }
        int check_up_area(vector< pair<int, float> > vec_degree)
        {
            for(int k = 0; k<vec_degree.size(); k++)
            {
                if(vec_degree[k].first > up_inspection_min && vec_degree[k].first < up_inspection_max)
                {
                    return 100;
                }
            }
            return 0;
        }
        
        // 검사 4가지 => 3가지 (뒤쪽 검사 필요없음), vec_pointxy(x, y)와 vec_degree(degree, range)의 size()는 같다.
        // 오른쪽 영역
        pair<float, float> right_area_inspection(vector< pair<float, float> > vec_pointxy, vector< pair<int, float> > vec_degree)
        {
            float foot_x = 0;
            float foot_y = 0;
            float foot_distance = 0;
            float highest_point_x = 0;
            float highest_point_y = 0;
            float move_point_x = 0;
            float move_point_y = 0;
            float max_degree = 0;

            pair<float, float> p;

            int count =0;

            for(int k=0; k<vec_degree.size(); k++)
            {
                if(vec_degree[k].first > right_inspection_min && vec_degree[k].first < right_inspection_max)
                {
                    // 가장 높은 위치의 점을 찾는다.
                    if(vec_degree[k].first > max_degree)
                    {
                        max_degree = vec_degree[k].first;
                        highest_point_x = vec_pointxy[k].first;
                        highest_point_y = vec_pointxy[k].second;                        
                    }

                     
                    // 89~91일때의 수선의 발을 찾는다.
                    if(vec_degree[k].first > right_foot_of_perpendicular_min && vec_degree[k].first < right_foot_of_perpendicular_max)
                    {
                        //수선의 발의 90도가 제일 정확. 89, 91은 차선책.
                        if(vec_degree[k].first == 90)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                        else if(vec_degree[k].first == 89 && count == 0)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                        else if(vec_degree[k].first == 91 && count == 0)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                    }
                }
            }
            // 오른쪽 영역에 점이 없을 때
            if(highest_point_x == 0 && highest_point_y == 0 && foot_x == 0 && foot_y == 0)
            {
                ROS_INFO("There is no point in right area");
                p = make_pair(0,0);
                return p;
            }
            // 90도로 꺾일 때, 즉, 90도를 회전해야 한다.
            if(highest_point_x == foot_x && highest_point_y == foot_y)
            {
                move_point_x = foot_x;
                move_point_y = foot_y + foot_distance;
                p = make_pair(move_point_x, move_point_y);
            }
            else // 사선으로 움직인다.
            {
                move_point_x = highest_point_x - foot_distance;
                move_point_y = highest_point_y;
                p = make_pair(move_point_x, move_point_y);
            }
            return p;
        }

        // 왼쪽 영역 , vec_pointxy(x, y)와 vec_degree(degree, range)의 size()는 같다.
        pair<float, float> left_area_inspection(vector< pair<float, float> > vec_pointxy, vector< pair<int, float> > vec_degree)
        {
            float foot_x = 0;
            float foot_y = 0;
            float foot_distance = 0;
            float highest_point_x = 0;
            float highest_point_y = 0;
            float move_point_x = 0;
            float move_point_y = 0;
            float min_degree = 999;

            pair<float, float> p;

            int count =0;

            for(int k=0; k<vec_degree.size(); k++)
            {
                if(vec_degree[k].first > left_inspection_min && vec_degree[k].first < left_inspection_max)
                {
                    // 가장 높은 위치의 점을 찾는다. (왼쪽 위이므로 degree가 가장 작은것.)
                    if(vec_degree[k].first < min_degree)
                    {
                        min_degree = vec_degree[k].first;
                        highest_point_x = vec_pointxy[k].first;
                        highest_point_y = vec_pointxy[k].second;                        
                    }

                     
                    // 268~272일때의 수선의 발을 찾는다.
                    if(vec_degree[k].first > left_foot_of_perpendicular_min && vec_degree[k].first < left_foot_of_perpendicular_max)
                    {
                        //수선의 발의 270도가 제일 정확. 269, 271은 차선책.
                        if(vec_degree[k].first == 270)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                        else if(vec_degree[k].first == 269 && count == 0)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                        else if(vec_degree[k].first == 271 && count == 0)
                        {
                            foot_x = vec_pointxy[k].first;
                            foot_y = vec_pointxy[k].second;
                            foot_distance = vec_degree[k].second;
                            count++;
                        }
                    }
                }
            }
            // 왼쪽 영역에 점이 없을 때
            if(highest_point_x == 0 && highest_point_y == 0 && foot_x == 0 && foot_y == 0)
            {
                ROS_INFO("There is no point in left area");
                p = make_pair(0,0);
                return p;
            }
            // 90도로 꺾일 때, 즉, -90도를 회전해야 한다.
            if(highest_point_x == foot_x && highest_point_y == foot_y)
            {
                move_point_x = foot_x;
                move_point_y = foot_y + foot_distance;
                p = make_pair(move_point_x, move_point_y);
            }
            else // 사선으로 움직인다.
            {
                move_point_x = highest_point_x + foot_distance;
                move_point_y = highest_point_y;
                p = make_pair(move_point_x, move_point_y);
            }
            return p;
        }
        
        pair<float, float> right_up_area_inspection(vector< pair<float, float> > vec_pointxy, vector< pair<int, float> > vec_degree)
        {
            float up_foot_x = 0;
            float up_foot_y = 0;
            float up_foot_distance = 0;

            int count = 0;

            pair< float , float > p;

            for(int k = 0; k<vec_degree.size(); k++)
            {
                // 179~181일때의 수선의 발을 찾는다.
                if(vec_degree[k].first > up_foot_of_perpendicular_min && vec_degree[k].first < up_foot_of_perpendicular_max)
                {
                    //수선의 발의 180도가 제일 정확. 179, 181은 차선책.
                    if(vec_degree[k].first == 180)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                    else if(vec_degree[k].first == 179 && count == 0)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                    else if(vec_degree[k].first == 181 && count == 0)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                }
            }
            // 위쪽에서 감지된 점의 수가 7개 이상이고 수선의 발이 있으며, 그 수선의 발의 거리가 0.3 이하일 때 
            ROS_INFO("up_foot_y : %f , up_foot_distance : %f", up_foot_y , up_foot_distance);
            if(up_foot_y > 0.001 && up_foot_distance < standard_distance)
            {
                ROS_INFO("up inspection the wall , perform turn left");
                fdmsg.turn_left = true;
                p = make_pair(0, 0);
                return p;
            }
            else // 다른경우에는 오른쪽 감지 방법 사용.
            {
                 p = right_area_inspection(vec_pointxy, vec_degree);
                 return p;
            }

            
        }

        pair<float, float> left_up_area_inspection(vector< pair<float, float> > vec_pointxy, vector< pair<int, float> > vec_degree)
        {
            float up_foot_x = 0;
            float up_foot_y = 0;
            float up_foot_distance = 0;

            int count = 0;

            pair< float , float > p;

            for(int k = 0; k<vec_degree.size(); k++)
            {
                // 179~181일때의 수선의 발을 찾는다.
                if(vec_degree[k].first > up_foot_of_perpendicular_min && vec_degree[k].first < up_foot_of_perpendicular_max)
                {
                    //수선의 발의 180도가 제일 정확. 179, 181은 차선책.
                    if(vec_degree[k].first == 180)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                    else if(vec_degree[k].first == 179 && count == 0)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                    else if(vec_degree[k].first == 181 && count == 0)
                    {
                        up_foot_x = vec_pointxy[k].first;
                        up_foot_y = vec_pointxy[k].second;
                        up_foot_distance = vec_degree[k].second;
                        count++;
                    }
                }
            }
            // 위쪽에서 감지된 점의 수가 20개 이상이고 수선의 발이 있으며, 그 수선의 발의 거리가 0.3 이하일 때 
            if(up_foot_y > 0.001 && up_foot_distance < standard_distance)
            {
                ROS_INFO("up inspection the wall , perform turn right");
                fdmsg.turn_right = true;
                p = make_pair(0, 0);
                return p;
            }
            else // 다른경우에는 왼쪽 감지 방법 사용.
            {
                 p = left_area_inspection(vec_pointxy, vec_degree);
                 return p;
            }

            
        }
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        Mat srcImage;
        Mat clear_Image;
        rplidar_ros::final_direction fdmsg;

        //degree 
        pair<int, float> p;
        vector< pair<int, float> > limited_ranges;
        
        //xy 좌표
        pair<float, float> p_xy;
        vector< pair<float, float> > pointxy_vec;
};

int main(int argc, char**argv)
{
    ros::init(argc, argv, "degree_to_pointxy");
    degree_to_pointxy d2p;
    ros::spin();
}