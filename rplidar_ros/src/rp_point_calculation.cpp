#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <utility>
#include "sensor_msgs/LaserScan.h"
#include "rplidar_ros/rp_point_calculation_define.h"
#include "rplidar_ros/new_scan.h"
#include "rplidar_ros/inspection_value.h"

using namespace std; // 성능 감소

class rp_point_calculation
{
    public:
        // 생성자
        rp_point_calculation()
        {
            sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1, &rp_point_calculation::getPoint_callback, this);
            pub = nh.advertise<rplidar_ros::inspection_value>("/inspection_value", 1000);
        }
        // 모든 point를 얻어오는 callback 함수
        void getPoint_callback(const sensor_msgs::LaserScanConstPtr &ptr)
        {
            range_min = ptr->range_min;
            range_max = ptr->range_max;
            
            for(int i=0; i<ptr->ranges.size(); i++)
            {
                // 일정 범위 내 값 추출.
                if(range_threshold_min < ptr->ranges[i] && ptr->ranges[i] < range_threshold_max)
                {
                    p =make_pair(i, ptr->ranges[i]);
                    limited_ranges.push_back(p);
                }
            }
            perform_inspection(limited_ranges);
            vector_clear();

            //실제 로봇 동작을 위한 처리를 하는 노드로 이동.
            pub.publish(ivmsg);
            clear_msg();
        }
        // vector의 값이 안들어갈때를 위해 값을 초기화해주는 함수.
        void vector_clear()
        {
            limited_ranges.clear();
        }
        
        // 검사 4가지
        int right_inspection(vector< pair<int, float> > vec)
        {
            for(int k=0; k<vec.size(); k++)
            {
                if(vec[k].first > right_inspection_min && vec[k].first < right_inspection_max)
                {
                    ROS_INFO("----- right inspection -----");
                    ivmsg.right_value = 1;
                    // 하나라도 감지되면 더이상 감지 할 필요가 없다.
                    return 1;
                } 
            }
        }
        int left_inspection(vector< pair<int, float> > vec)
        {
            for(int k=0; k<vec.size(); k++)
            {
                if(vec[k].first > left_inspection_min && vec[k].first < left_inspection_max)
                {
                    ROS_INFO("----- left inspection -----");
                    ivmsg.left_value = 10;
                    return 1;
                } 
            }
        }
        int up_inspection(vector< pair<int, float> > vec)
        {
            for(int k=0; k<vec.size(); k++)
            {
                if(vec[k].first > up_inspection_min && vec[k].first < up_inspection_max)
                {
                    ROS_INFO("----- up inspection -----");
                    ivmsg.up_value = 100;
                    return 1;
                } 
            }
        }
        int down_inspection(vector< pair<int, float> > vec)
        {
            for(int k=0; k<vec.size(); k++)
            {
                if(vec[k].first > down_inspection_min && vec[k].first < down_inspection_max + 360)
                {
                    ROS_INFO("----- down inspection -----");
                    ivmsg.down_value = 1000;
                    return 1;
                } 
            }
        }
        //four inspection perform
        void perform_inspection(vector< pair<int, float> > vec)
        {
            right_inspection(vec);
            left_inspection(vec);
            up_inspection(vec);
            down_inspection(vec);
        }

        void clear_msg()
        {
            ivmsg.right_value = 0;
            ivmsg.left_value = 0;
            ivmsg.up_value = 0;
            ivmsg.down_value = 0;
        }
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        rplidar_ros::inspection_value ivmsg;
        float range_min; // m
        float range_max; // m

        pair<int, float> p;
        vector< pair<int, float> > limited_ranges;
        //vector<float> intensities;
};

int main(int argc, char*argv[])
{
    ros::init(argc, argv, "first_node_to_get_all_point");
    rp_point_calculation rp_p_c;
    ros::spin();
}


/* sensor_msgs/LaserScan 구조
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
*/

// lidar의 뾰족한 부분을 앞으로 놓았을 때 뒤 꽁무늬가 0도, 앞이 180도 반시계방향으로 1도씩 증가.