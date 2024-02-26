#include <vector>
#include <cmath>
#include "arg.hh"
#include "table.hh"
#define M_PI		3.14159265358979323846
using namespace std;
char mp[100][100];

 int wall_sum = 0;
const double robot_as_obs_dis = 4;
const double eps = 0.1;
const double max_lspeed = 7;
const int sum_time = 12000;
const int table_type_num = 9;
const int product_type_num = 7;
const double max_rspeed = M_PI;
// const double diameter_robot_without = 0.54;
const double diameter_robot_with = 2;
const int price[] = 	{0, 6000, 7600, 9200, 22500, 25000, 27500, 105000};	//i号产品售价
const int cost[] = 		{0, 3000, 4400, 5800, 15400, 17200, 19200, 76000};	//i号产品单价
vector<int> profit_back = 	{1, 3000, 3001, 3002, 6001,  6002,  6003,  29000};	//i号产品利润
vector<int> buy_profit = 		{1, 3000, 3001, 3002, 6001,  6002,  6003,  29000};	//i号产品利润
vector<int> sell_profit = 		{1, 3000, 3001, 3002, 6001,  6002,  6003,  29000};	//i号产品利润
vector<int> product_time = 	{0, 50, 50, 50, 500, 500,  500,  1000};	//i号产品生产时间
const int need_time[] = {100, 50, 50, 50, 500,  500,  500,  1000};	//i号产品利润
vector<bool> seven_need(7, false);
vector<int> diff_table_num(10, 0);
const vector<vector<int>> the_table_want_buy = {
						{},
						{}, 	//1
						{},		//2
						{},		//3
						{1,2},	//4
						{1,3},	//5
						{2,3},	//6
						{4,5,6},//7
						{7},	//8
						{1,2,3,4,5,6,7},//9
					};
char line[1024];
int frameID, money;
vector<vector<double>> robots_speed(4, vector<double>(2, 0));
vector<int> product_num(product_type_num+1, 0);

int seven_need_component_priority = 3; 
int countdown = 1500;
double crash_threshold = 60;
double position_arg = 1.0;
double slow_down = 10;
double speed_expectation = 4; 
double buy_min_distance = 0;	
double sell_min_distance = 6.0;	
double wait_punish = 800; 
double buy_close456 = 16;
double sell_same_place = 2;
double pre_task = 10000;
int meantime_buy = 2;
bool no_seven_mode = false;

vector<vector<shared_ptr<Path>>> path_bettwen_tables(51, vector<shared_ptr<Path>>(51, nullptr));
vector<vector<shared_ptr<Path>>> path_bettwen_tables_take(51, vector<shared_ptr<Path>>(51, nullptr));
vector<vector<bool>> robots_to_tables(4, vector<bool>(51, false));
vector<vector<shared_ptr<Path>>> path_bettwen_robots_and_tables(4, vector<shared_ptr<Path>>(51, nullptr));
vector<vector<int>> dir = {{1,0}, {-1,0}, {0,1}, {0,-1}, {1,-1}, {1,1}, {-1,-1}, {-1, 1}}; 
vector<vector<int>> dir2 = {{-2,-2}, {-2,-1}, {-2, 0}, {-2,1}, {-2,2},
								{2,-2}, {2,-1}, {2, 0}, {2,1}, {2,2},
								{-1, 2}, {0, 2}, {1, 2},
								{-1, -2}, {0, -2}, {1, -2}};
bool red;

vector<Table*> block_list;

Enemy::Enemy(double x, double y, double r) : x(x), y(y), r(r) {}
Radar_line::Radar_line(double x, double y, double d) : x(x), y(y), d(d) {}

vector<Enemy> enemys;
