#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector> // vector
#include <math.h> // abs
#include "constant_a.h" // 상수가 정의되어있는 헤더파일
#include "autodrive/autodrive.h" // msg 가 정의되어있는 함수.
#include <queue>

using namespace std;
using namespace cv;

Mat region_of_interest(Mat canny_image); // ROI 
Mat hough_lines(Mat ROI_image, Mat srcImage_COLOR); 
Mat weight_image(Mat hough_image, Mat srcImage);

void draw_lines(Mat ROI_image, vector<Vec4i> lines, Mat srcImage_COLOR); 
void Draw_fitLine_L(Mat line_image, Vec4f output);
void Draw_fitLine_R(Mat line_image, Vec4f output); 
void print_Line_and_point(Mat line_image, vector<Point2f> Pointxy, int L);
void calc_x_y_point(Mat line_image, float vx, float vy, float x, float y, int L); 
void transform_point(Mat line_image);
void print_center_point(Mat line_image);
void cal_car_radian(Mat line_image);

int L_center_x, L_center_y = 0;
int R_center_x, R_center_y = 0;
int center_x;
int center_y;
double radian = 0;

queue<pair<int, int>> center_x_y_queue;
queue<pair<int, int>> center_x_y_queue_sub;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "autodrive_pub");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<autodrive::autodrive>("opencv_msg",1000);
	autodrive::autodrive msg;
	msg.goto_radian = 0;
	msg.goto_center_x = 0;
	msg.goto_center_y = 0;

	VideoCapture cap("soongsil_drive.avi"); // 동영상 출력할때 사용.
	//VideoCapture inputVideo(0); //웹캠으로 출력할때 사용.
	cv::Mat frame;
	Mat srcImage_GRAY;
	Mat srcImage_COLOR;
	Mat srcImage;
	Mat hough_image;
	Mat canny_image;
	Mat ROI_image;
	Mat result;
	int n = 1;
	int m = 1;

	//int fps = (int)(inputVideo.get(CAP_PROP_FPS));
	//int delay = Delay / fps;

	while(ros::ok())
	{
		cap >> frame; // frameÀ» ÀÐŸî¿ÂŽÙ.
		srcImage_COLOR = frame.clone(); // ¿øº»ÀÌ¹ÌÁö º¹»ç(ÄÃ·¯)
		cv::cvtColor(frame, srcImage_GRAY, COLOR_BGR2GRAY); //Èæ¹éÀž·Î º¯°æ
		Canny(srcImage_GRAY, canny_image, canny_threshold_1, canny_threshold_2, apertureSize); // edge °ËÃâ
		ROI_image = region_of_interest(canny_image); // ROI
		hough_image = hough_lines(ROI_image, srcImage_COLOR);
		result = weight_image(hough_image, srcImage_COLOR);
		imshow("result", result);

		msg.goto_center_x = center_x; // 메세지 값 넣기
		msg.goto_center_y = center_y; // 메세지 값 넣기
		msg.goto_radian = radian;// 라디안 넣기

		//보내는 값 확인
		ROS_INFO("center x : %d", msg.goto_center_x);
		ROS_INFO("center y : %d", msg.goto_center_y);
		ROS_INFO("radian : %f", msg.goto_radian);

		pub.publish(msg);
		int ckey = waitKey(30);
		if (ckey == ESC) break;
	}
	return 0;
}


Mat region_of_interest(Mat canny_image) 
{
	int height = canny_image.size().height;
	int width = canny_image.size().width;
	Mat dst_image;
	Mat mask(canny_image.rows, canny_image.cols, CV_8UC1, Scalar(0));
	Point pts[6] = { Point(0,height),Point(0, height / 3 * 2), Point(width / 3, height / 2), Point(width / 3 * 2 , height / 2), Point(width, height / 3 * 2), Point(width,height) };
	fillConvexPoly(mask, pts, 6, Scalar(255));
	bitwise_and(canny_image, mask, dst_image);
	return dst_image;
}

Mat hough_lines(Mat ROI_image, Mat srcImage_COLOR)
{
	Mat line_image(ROI_image.size(), CV_8UC3, Scalar(0, 0, 0));

	vector<Vec4i> lines;
	HoughLinesP(ROI_image, lines, rho, delta, hough_threshold, MinLineLength, MaxLineGap);
	draw_lines(line_image, lines, srcImage_COLOR);

	return line_image;
}

Mat weight_image(Mat hough_image, Mat srcImage)
{
	Mat outMat;
	addWeighted(srcImage, alpha, hough_image, beta, gamma, outMat);
	return outMat;
}

void print_Line_and_point(Mat line_image, vector<Point2f> Pointxy, int L)
{
	if (Pointxy.size() != 0)
	{
		Vec4f output;

		fitLine(Pointxy, output, DIST_L2, fit_param, fit_reps, fit_aeps); 
		if (L == Left) Draw_fitLine_L(line_image, output); 
		if (L == Right) Draw_fitLine_R(line_image, output); 

		Point_<float> pt(output[2], output[3]);
		circle(line_image, pt, radius, Scalar(0, 255, 0), line_thickness);
		if (L == Right)
		{
			//cout << "Right ( " << output[2] << " , " << output[3] << " ) " << endl;
			R_center_x = output[2];
			R_center_y = output[3];
		}

		else if (L == Left) // ¿ÞÂÊ
		{
			//cout << "Left ( " << output[2] << " , " << output[3] << " ) " << endl;
			L_center_x = output[2];
			L_center_y = output[3];
		}
	}
}

void Draw_fitLine_L(Mat line_image, Vec4f output)
{
	float vx = output[0];
	float vy = output[1];
	float x = output[2];
	float y = output[3];

	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = (int)(x1 - 200 * vx);
	int y2 = (int)(y1 - 200 * vy);

	Point_<int> pt1(x1, y1), pt2(x2, y2);
	line(line_image, pt1, pt2, Scalar(255, 255, 255), line_thickness); 
	//cout << "L slope_degree : " <<(-1)* vy/vx * 180 / CV_PI << endl;
	//cout << "L_vx = " << vx << ", L_vy = " << vy;
	calc_x_y_point(line_image, vx, vy, x, y, Left);
}

void Draw_fitLine_R(Mat line_image, Vec4f output)
{
	float vx = output[0];
	float vy = output[1];
	float x = output[2];
	float y = output[3];

	int x1 = (int)x;
	int y1 = (int)y;
	int x2 = (int)(x1 + 200 * vx);
	int y2 = (int)(y1 + 200 * vy);

	Point_<int> pt1(x1, y1), pt2(x2, y2);
	line(line_image, pt1, pt2, Scalar(255, 255, 255), line_thickness);
	//cout << "R slope_degree : " <<(-1)* vy / vx *180 /CV_PI << endl;
	//cout << "R_vx = " << vx << ", R_vy = " << vy;
	calc_x_y_point(line_image, vx, vy, x, y, Right);
}

void draw_lines(Mat line_image, vector<Vec4i> lines, Mat srcImage_COLOR)
{
	Vec4i params;

	vector<Point2f> L_Pointxy;
	vector<Point2f> R_Pointxy;

	int x1, y1, x2, y2;
	double slope_degree;
	int height = line_image.size().height;
	int width = line_image.size().width;

	for (int k = 0; k < lines.size(); k++) 
	{
		params = lines[k];
		x1 = params[0];
		y1 = params[1];
		x2 = params[2];
		y2 = params[3];
		Point_<int> pt1(x1, y1), pt2(x2, y2);
		slope_degree = atan2(y2 - y1, x2- x1) * 180 /CV_PI;

		if (abs(slope_degree) < Max_slope_degree && abs(slope_degree) > Min_slope_degree) 
		{
			if (slope_degree > inclination_standard)
			{
				line(line_image, pt1, pt2, Scalar(255, 0, 0), line_thickness);
				R_Pointxy.push_back(pt1);
				R_Pointxy.push_back(pt2);

			}
			else if (slope_degree < inclination_standard)
			{
				line(line_image, pt1, pt2, Scalar(255, 0, 0), line_thickness);
				L_Pointxy.push_back(pt1);
				L_Pointxy.push_back(pt2);
			}
		}
	}
	print_Line_and_point(line_image, R_Pointxy, Right); 
	print_Line_and_point(line_image, L_Pointxy, Left);

	print_center_point(line_image); 
}

void calc_x_y_point(Mat line_image,float vx, float vy, float x, float y, int L)
{
	float inclination = vy / vx; 
	int width = line_image.size().width;
	int height = line_image.size().height;
	int y_point = (int)(y - inclination * x);

	int x_down;
	int y_down;
	int x_up;
	int y_up;

	if (L == Left)
	{
		if (y_point > height)
		{
			x_down = (int)((height - y_point) / inclination);
			y_down = height;
		}
		else
		{
			x_down = 0;
			y_down = y_point;
		}
		if ((height / 2 - y_point) / inclination > width) 
		{
			x_up = width;
			y_up = (int)(inclination * width + y_point);
		}
		else
		{
			x_up = (int)((height / 2 - y_point) / inclination);
			y_up = height / 2;
		}
		Point2f pt1(x_down, y_down), pt2(x_up, y_up);
		circle(line_image, pt1, 3, Scalar(0, 0, 255), 3);
		circle(line_image, pt2, 3, Scalar(0, 0, 255), 3);
	}
	else if (L == Right)
	{
		if (inclination * width + y_point > height)
		{
			x_down = (int)((height - y_point) / inclination);
			y_down = height;
		}
		else 
		{
			x_down = width;
			y_down = (int)(inclination * width + y_point); 
		}
		if ((height / 2 - y_point) / inclination < 0) 
		{
			x_up = 0;
			y_up = y_point;
		}
		else
		{
			x_up = (int)((height / 2 - y_point) / inclination);
			y_up = height / 2;
		}

		Point pt3(x_down, y_down), pt4(x_up, y_up);
		circle(line_image, pt3, 3, Scalar(0, 0, 255), 3);
		circle(line_image, pt4, 3, Scalar(0, 0, 255), 3);
	}
}

void print_center_point(Mat line_image)
{
	int height = line_image.size().height;
	int width = line_image.size().width;

	center_x = (L_center_x + R_center_x) / 2;
	center_y = (L_center_y + R_center_y) / 2;

	pair<int, int> p;

	if (center_x_y_queue.size() < 20)
	{
		p = make_pair(center_x, center_y);
		center_x_y_queue.push(p);
	}
	else
	{
		int sum_x = 0;
		int sum_y = 0;
		int count_queue = 0;
		while (center_x_y_queue.size() != 0)
		{
			p = center_x_y_queue.front();
			sum_x += p.first;
			sum_y += p.second;
			center_x_y_queue.pop();
			if (count_queue != 0)
			{
				center_x_y_queue_sub.push(p);
			}
			count_queue++;
		}
		center_x = sum_x / 20;
		center_y = sum_y / 20;
		
		cal_car_radian(line_image); // 각도 추출
		
		Point pt(center_x, center_y);
		Point pt2(width / 2, height / 2);
		Point pt3(width / 2, height);

		circle(line_image, pt, 5, Scalar(100, 100, 100), 3);
		line(line_image, pt3, pt2, Scalar(200, 200, 200), 3);

		while (center_x_y_queue_sub.size() != 0)
		{
			p = center_x_y_queue_sub.front();
			center_x_y_queue_sub.pop();
			center_x_y_queue.push(p);
		}
	}
}

void cal_car_radian(Mat line_image)
{
	int height = line_image.size().height;
	int width = line_image.size().width;
	if(center_x >= width/2) radian = atan2(center_x - width/2 , height - center_y) * 180 /CV_PI;
	else radian = atan2(width /2 - center_x , height - center_y) * 180 / CV_PI * (-1);
	//cout <<"move radian : "<< radian << endl;
}