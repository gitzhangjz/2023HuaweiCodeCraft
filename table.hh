#ifndef TABLE_HH
#define TABLE_HH
#include <vector>
using namespace std;

struct Table {
	int 	id, 				//桌子id
			table_type, 		//桌子类型
			remain_time, 		//生产剩余时间
			have_product, 		//产品状态
			product_type, 		//产品类型
			remain_component,	//还需要几个配件
			sum_component,		//需要配件总数
			shut_down,
			occupation;
	double 	x, 					//桌子坐标x
			y,					//桌子坐标y
			hit_x,
			hit_y;
	vector<bool> want_buy;
	vector<bool> hava_this_component;

	Table (int id, int table_type, int product_type, int sum_component, double x, double y);


	//设置桌子状态
	void set_state(double remain_time, int component_state, int have_product);
};
extern vector<Table> tables;
extern vector<Table> d_tables;

#endif