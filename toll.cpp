#include "toll.hh"
#include "arg.hh"
#define M_PI		3.14159265358979323846
#include<cmath>

double distance_bettwen_two(const double &dx, const double &dy, const double &x, const double &y)
{
	return sqrt((dy-y)*(dy-y) + (dx-x)*(dx-x));
}

//theta1转到theta2所需的角度，返回值：[-pi, pi]，逆时针为正，顺时针为负。
double angle(double theta1, double theta2)
{
	double new_speed_angle = theta1 + (theta1 < 0 ? 2*M_PI : 0);
	double new_dir_i = theta2 + (theta2 < 0 ? 2*M_PI : 0);

	double reverse_clock, clock;

	if(new_speed_angle < new_dir_i)
	{
		reverse_clock = new_dir_i - new_speed_angle;
		clock = new_speed_angle + 2*M_PI - new_dir_i;
	}else {
		reverse_clock = 2*M_PI - new_speed_angle + new_dir_i;
		clock = new_speed_angle - new_dir_i;
	}

	if(reverse_clock < clock)
		return reverse_clock;
	else
		return -1*clock;
}


// 计算direction2在direction1的投影长度
double projection(double d1, double d2, double x) {
    return x * cos(d2 - d1);  
}

double time_lose(double x)
{
	return (1-sqrt(1-pow(1-x/sum_time, 2))) * (1-0.8) + 0.8;
}

bool tree_point_circular(double x1, double y1, double x2, double y2, double x3, double y3, double &r, double &x, double &y)
{
    double x_2_y_3 = x2*y3;
    double x_3_y_2 = x3*y2;
 
    double x2_x3 = x2-x3;
    double y2_y3 = y2-y3;
 
    double x_1_x_1py_1_y_1 = x1*x1 + y1*y1;
    double x_2_x_2py_2_y_2 = x2*x2 + y2*y2;
    double x_3_x_3py_3_y_3 = x3*x3 + y3*y3;
 
    double A = x1 * y2_y3 - y1 * x2_x3 + x_2_y_3 - x_3_y_2;
    double B = x_1_x_1py_1_y_1 * (-y2_y3) + x_2_x_2py_2_y_2 * (y1-y3) + x_3_x_3py_3_y_3 * (y2-y1);
    double C = x_1_x_1py_1_y_1 * x2_x3 + x_2_x_2py_2_y_2 * (x3 - x1) + x_3_x_3py_3_y_3 * (x1-x2);
    double D = x_1_x_1py_1_y_1 * (x_3_y_2 - x_2_y_3) + x_2_x_2py_2_y_2 * (x1*y3 - x3*y1) + x_3_x_3py_3_y_3 * (x2*y1-x1*y2);
 
    x = -B / (2*A);
    y = -C / (2*A);
    r=sqrt((B*B + C*C - 4*A*D) / (4*A*A));
    //圆不存在
    if(abs(A) < 1e-5 || (abs(r-0.45)>1e-5 && abs(r-0.53)>1e-5) || x < 0 || x > 50 || y < 0 || y > 50)
        return false;
    else
		return true;
}

double point_to_line_segment_distance(double p_x, double p_y, double start_x, double start_y,double end_x, double end_y) {
    // 计算线段的长度
    double length = distance_bettwen_two(start_x, start_y, end_x, end_y);

    // 如线段长度为0，返回点p到起点的距离
    if (length == 0) {
        return distance_bettwen_two(p_x, p_y, start_x, start_y);
    }

    // 计算向量V1和V2，分别为起点到点p的向量和起点到终点的向量
	double v1_x = p_x - start_x, v1_y = p_y - start_y,
			v2_x = end_x - start_x, v2_y = end_y - start_y;

    // 计算点p在向量V2上的投影点
    double mag = distance_bettwen_two(v2_x, v2_y, 0, 0);
    double scalar = ((v1_x * v2_x) + (v1_y * v2_y)) / pow(mag, 2.0);

	double projection_x = scalar * v2_x, projection_y = scalar * v2_y;

    // 如果投影点在线段的起点左侧或终点右侧，则返回点p到这两个点中距离较近的那个距离
    if (projection_x < 0) {
        return distance_bettwen_two(p_x, p_y , start_x, start_y);
    } else if (projection_x > length) {
        return distance_bettwen_two(p_x, p_y, end_x, end_y);
    }

    // 计算投影点到点p的距离，即为点p到线段的最短距离
    return distance_bettwen_two(p_x, p_y, start_x + projection_x, start_y + projection_y);
}
