#include <ros/ros.h>
#include "object_manager/ROIcheck.h"

// 인자로 현재 위치 x, y, 한 미션에 대한 polygon 정보.
// 현재 자동차의 위치가 polygon 내부에 있는지 확인하는 함수?? polygon 내부에 차량이 있어야 하는 것인가??
bool isPointInPolygon(double x, double y, const geometry_msgs::Polygon& poly){
    struct Pos{
        double x, y;
    };
    // 람다 [introducer capture](parameters)->(return type) {return x}
    // 이 함수는 삼각형의 꼭짓점 3개가 주어졌을 때 넓이를 구하는 함수로, 
    /*
        S = fabs((x1*y2 + x2*y3 + x3*y1) - (x1*y3 + x3*y2 + x2*y1)) / 2 
        이 식을 정리하여 얻은 식이 밑의 area 구하는 식이다. 
     */
    // 세 점의 좌표를 이용해 삼각형의 넓이를 구하는 식.
    static auto calc_triangle_area = [](const Pos& f,const Pos& s,const Pos& t)->double { //first, second, third
        double area = fabs((f.x*(s.y-t.y) + s.x*(t.y-f.y) + t.x*(f.y-s.y))/2);
        return area;
    };
    //엡실론 , 부동소수점 연산에서 반올림을 함으로써 발생하는 오차의 상한.
    static double EPSILON = 1;

    if (!poly.points.size()) return false;

    Pos p{x,y};
    const int N_POINT = (int)poly.points.size(); // point 개수 저장.
    double sum_of_poly = 0;
    double sum_of_poly_with_point = 0;

    //calc sum of poly
    Pos t_anchor{poly.points.front().x, poly.points.front().y}; // poly 좌표중 가장 앞에있는것 객체로 만듦
    // 처음과 끝좌표를 제외하고 계산. 위 코드를 통해 첫좌표와 끝좌표를 빼온것.
    for(int i = 1; i < N_POINT - 1; ++i){
        Pos t1{poly.points[i].x, poly.points[i].y};
        Pos t2{poly.points[i+1].x, poly.points[i+1].y};
        sum_of_poly += 
            calc_triangle_area(t_anchor, t1, t2);// 한 점을 기준으로 부분 삼각형의 합을 구하여 최종적으로 polygon의 넓이를 구한다.
    }

    //calc sum of poly with point
    //처음과 끝의 좌표는 동일하다. 원 구조이기 때문.
    for(int i = 0 ; i < N_POINT; ++i){
        Pos t1{poly.points[i % N_POINT].x, poly.points[i % N_POINT].y};
        Pos t2{poly.points[(i + 1) % N_POINT].x, poly.points[(i + 1) % N_POINT].y};
        sum_of_poly_with_point += 
            calc_triangle_area(p, t1, t2); // 현재 좌표. poly의 연속된 두 개의 좌표 
    }    

    if (fabs(sum_of_poly - sum_of_poly_with_point) <= EPSILON) return true; //the point is in first triangle
    else return false;
}