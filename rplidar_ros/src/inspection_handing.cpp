#include<ros/ros.h>
#include<iostream>
#include<vector>
#include<utility>
#include<stdlib.h>
#include"rplidar_ros/inspection_value.h"
#include"rplidar_ros/rp_point_calculation_define.h"
#include"rplidar_ros/final_direction.h"

using namespace std;

class inspection_handling{
    public:
        inspection_handling() :right_value(false), left_value(false), up_value(false), down_value(false)
        {
            sub = nh.subscribe<rplidar_ros::inspection_value>("/inspection_value", 1, &inspection_handling::direct_decision_callback, this);
            pub = nh.advertise<rplidar_ros::final_direction>("/final_direction", 1000);
        }

        void direct_decision_callback(const rplidar_ros::inspection_valueConstPtr &ptr)
        {
            right_value = ptr->right_value;
            left_value = ptr->left_value;
            up_value = ptr->up_value;
            down_value = ptr->down_value;
            
            total_value = right_value + left_value + up_value + down_value;
            ROS_INFO("total_value = %d", total_value);

            direct_decision(total_value);

            pub.publish(fdmsg);

            clean_value();

        }

        void direct_decision(int total_value)
        {
            fdmsg.go_straight = false;
            fdmsg.turn_right = false;
            fdmsg.turn_left = false;
            fdmsg.turn_back = false;

            switch(total_value)
            {
                case zero_:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case right_:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case left_:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case up_:
                    fdmsg.turn_right = true;
                    ROS_INFO("turn_right !!!");
                    break;
                case down_:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case right_left:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case right_up:
                    fdmsg.turn_left = true;
                    ROS_INFO("turn_left !!!");
                    break;
                case right_down:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case right_left_up:
                    fdmsg.turn_back = true;
                    ROS_INFO("turn_back !!!");
                    break;
                case right_left_down:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case right_left_up_down:
                    ROS_INFO(" be locked in a room !!!!");
                    exit(1);
                    break;
                case right_up_down:
                    fdmsg.turn_left = true;
                    ROS_INFO("turn_left !!!");
                    break;
                case left_up:
                    fdmsg.turn_right = true;
                    ROS_INFO("turn_right !!!");
                    break;
                case left_down:
                    fdmsg.go_straight = true;
                    ROS_INFO("go straight!!!");
                    break;
                case left_up_down:
                    fdmsg.turn_right = true;
                    ROS_INFO("turn_right !!!");
                    break;
                case up_down:
                    fdmsg.turn_right = true;
                    ROS_INFO("turn_right !!!");
                    break;      
            }
        }

        void clean_value()
        {
            right_value = 0;
            left_value = 0;
            up_value = 0;
            down_value = 0;
            total_value = 0;
        }
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;

        rplidar_ros::inspection_value ivmsg;
        rplidar_ros::final_direction fdmsg;

        int right_value;    // 1
        int left_value;     // 10
        int up_value;       // 100
        int down_value;     // 1000
        int total_value;
};

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "inspection_handling");
    inspection_handling ih;
    ros::spin();
}

