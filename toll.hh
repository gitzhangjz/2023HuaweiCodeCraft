// 一些功能函数
#ifndef TOLL_HH
#define TOLL_HH
#include <iostream>
using namespace std;
// 输出调试信息
#define FUCK(X) cerr<<(#X)<<": "<<X<<endl;

// 两点距离
double distance_bettwen_two(const double &dx, const double &dy, const double &x, const double &y);
// 从theta1转到theta2的角度
double angle(double theta1, double theta2);

//三点确定一个圆
bool tree_point_circular(double x1, double y1, double x2, double y2, double x3, double y3, double &r, double &x, double &y);

//没有用到
double projection(double d1, double d2, double x);
double time_lose(double x);
double point_to_line_segment_distance(double p_x, double p_y, double start_x, double start_y,double end_x, double end_y);

#endif
