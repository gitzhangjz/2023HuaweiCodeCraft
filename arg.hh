// 一些全局变量
#ifndef ARG_HH
#define ARG_HH

#include <vector>
#include "A_star.hh"
#include "table.hh"
using namespace std;

extern char mp[100][100];

extern int wall_sum; // 当前地图墙的数量
extern const double robot_as_obs_dis;
extern const double eps;
extern const double max_lspeed;
extern const int sum_time;
extern const int table_type_num;
extern const int product_type_num;
extern const double max_rspeed;
// const double diameter_robot_without = 0.54;
extern const double diameter_robot_with;
extern const int price[];	//i号产品售价
extern const int cost[];	//i号产品单价
extern vector<int> profit_back;	//i号产品利润
extern vector<int> buy_profit;	//i号产品利润
extern vector<int> sell_profit;	//i号产品利润
extern vector<int> product_time;	//i号产品生产时间
extern const int need_time[];	//i号产品利润
extern vector<bool> seven_need;
extern vector<int> diff_table_num;
extern const vector<vector<int>> the_table_want_buy;
extern char line[1024];
extern int frameID, money;
extern vector<vector<double>> robots_speed;
extern vector<int> product_num;

extern int seven_need_component_priority;
extern int countdown;
extern double crash_threshold;
extern double position_arg;
extern double slow_down;
extern double speed_expectation;
extern double buy_min_distance;
extern double sell_min_distance;
extern double wait_punish;
extern double buy_close456;
extern double sell_same_place;
extern double pre_task;
extern int meantime_buy;

extern vector<vector<shared_ptr<Path>>> path_bettwen_tables;
extern vector<vector<shared_ptr<Path>>> path_bettwen_tables_take;
extern vector<vector<shared_ptr<Path>>> path_bettwen_robots_and_tables;
extern vector<vector<bool>> robots_to_tables;
extern vector<vector<int>> dir; 
extern vector<vector<int>> dir2;
extern bool no_seven_mode;
extern bool red;

extern vector<Table*> block_list;

struct Enemy{
	double x,y,r;
	Enemy(double x, double y, double r);
};

struct Radar_line{
	double x,y,d;
	Radar_line(double x, double y, double d);
};

extern vector<Enemy> enemys;
#endif