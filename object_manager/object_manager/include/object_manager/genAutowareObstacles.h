#include <visualization_msgs/MarkerArray.h>
#pragma once
void genObstacleMsgFromBoundingbox(autoware_msgs::DetectedObject& output,
    const jsk_recognition_msgs::BoundingBox& input);
void genObstacleMsgFromRectanglePoly(autoware_msgs::DetectedObject& output,
    const geometry_msgs::Polygon& poly, double gap_between_polygon_point);
void genObstacleMsgFromPolygon(autoware_msgs::DetectedObject& output,
    const geometry_msgs::Polygon& poly, double gap_between_polygon_point);
void addDebugMarkersFromObstacles(visualization_msgs::MarkerArray& markers,
    const autoware_msgs::DetectedObject& obj);