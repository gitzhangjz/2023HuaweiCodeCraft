#include <iostream>
#include <vector>
#include <cmath>
#include <queue>
#include <fstream>
#include <algorithm>
#include <random>
#include "arg.hh"
#include "robot.hh"
#include "toll.hh"
#include "A_star.hh"
using namespace std;

// 输出对方机器人的调试信息
void debug_enemy()
{
	FUCK(frameID)
	FUCK(enemys.size())
	if(enemys.size())
	{
		for(auto &e : enemys)
		{
			FUCK(e.x)
			FUCK(e.y)
			FUCK(e.r)
			cerr << endl;
		}
	}
	cerr << endl;
}

// 输出自己机器人的调试信息，i为机器人编号
void debug_robot(int i)
{
	Robot &r = robots[i];
	FUCK(frameID)
	FUCK(r.id)
	FUCK(r.shut_down)
	FUCK(r.slow_speed)
	FUCK(r.lock_time)
	// FUCK(r.in_table_id)
	FUCK(r.tar_x)
	FUCK(r.tar_y)
	FUCK(robots_speed[r.id][0])
	FUCK(robots_speed[r.id][1])
	if(r.target_table)
	{
		FUCK(r.target_table->table_type)
		FUCK(r.target_table->x)
		FUCK(r.target_table->y)
		if(r.path)
		{
			FUCK(r.path->cost)
			FUCK(r.follow_indx)
			for(auto p : r.path->nodes)
			{
				cerr << (p->x)/2.0+0.25 << ',' << (p->y)/2.0+0.25 << endl;
			}
		}
		// FUCK(r.path->cost)
	}else{
		FUCK("NO")
	}
	cerr << endl;
}

// 寻找最近的一个工作台收购this_table工作台的产品，返回最近工作台的距离
double closest_buyer(Table& this_table)
{
	if(this_table.table_type >= 8)
		return 0;
	double min_d = 200;
	for(auto &t : tables)
	{
		if(t.shut_down > 0) {
			continue;
		}
		if(t.table_type == 9 || (t.table_type == 8 && this_table.table_type == 7))
		{
			if( path_bettwen_tables_take[this_table.id][t.id])
				min_d = min(min_d, (double)path_bettwen_tables_take[this_table.id][t.id]->cost);
			continue;
		}
		if(!(t.want_buy[this_table.product_type] && 
			!t.hava_this_component[this_table.product_type]) || 
			(robots[0].take_type == this_table.table_type && robots[0].target_table == &t) ||
			(robots[1].take_type == this_table.table_type && robots[1].target_table == &t) ||
			(robots[2].take_type == this_table.table_type && robots[2].target_table == &t) ||
			(robots[3].take_type == this_table.table_type && robots[3].target_table == &t)
		)
			continue;
		if(path_bettwen_tables_take[this_table.id][t.id])
			min_d = min(min_d, (double)path_bettwen_tables_take[this_table.id][t.id]->cost);
		
		if(t.occupation > 0)
			min_d *= 5;
	}
	return min_d;
}

// 优先级队列的节点结构
struct q_node {
	double profit;
	Robot *r;
	Table *t;
	q_node(double profit, Robot* r, Table *t) : profit(profit), r(r), t(t) {}

	bool operator<(const q_node& a) const {return profit < a.profit; } 
};

//修正因为跳帧产生的错误
void fix_frameskip(Robot &rbt)
{
	if((rbt.take_type == 0 && rbt.task_type == 1) || 
		  (rbt.take_type != 0 && rbt.task_type == 0) ||
		  (rbt.task_type == -1) ||
		  (rbt.take_type == 0 && rbt.target_table && rbt.target_table->have_product==false && rbt.target_table->remain_time == -1) ||
		  (rbt.take_type != 0 && rbt.target_table && rbt.target_table->hava_this_component[rbt.take_type]))
	{
		rbt.clear();
	}
}

// 计算rbt购买this_table产品的优先级并加入任务队列
void buy(Robot &rbt, Table &this_table, priority_queue<q_node>& buy_tasks)
{
	//没有产品并且还没有生产
	if(this_table.have_product == 0 && this_table.remain_time == -1)
		return;
	//生产时间太长
	if(this_table.have_product == 0 && this_table.remain_time > 100)
		return;
	
	// 购买物品后卖给其他工作台的最近距离
	double min_d = closest_buyer(this_table);

	double distance, profitt = 0, time = 0;
	
	// 判定机器人和工作台之间有无路径
	bool have_path = path_finder.run(rbt.x, rbt.y, this_table.x, this_table.y, false, 80);
	
	if(have_path)
	{
		distance = path_finder.path->cost;
	}else{	//机器人和this_table之间没有路径
		return;
	}

	double 	go_time = distance/speed_expectation*50,
			sell_time = min_d/speed_expectation*50,
			wait_time = this_table.have_product ? 0 : this_table.remain_time - go_time;
			wait_time = max(0.0, wait_time);
	
	// 工作台被对方机器人占领，降低优先级
	if(this_table.occupation)
		go_time *= 5;
	
	// 机器人附近有对方机器人，降低优先级
	if(rbt.close_enemy > 0)
		go_time *= 5;

	//时间不够送不到的情况
	if(frameID + go_time + sell_time + wait_time >= sum_time) 
		return;

	time = go_time + sell_time + wait_time;
	profitt = buy_profit[this_table.product_type];

	// 当时间快结束的时候，优先买大号物品
	if(sum_time-frameID <= countdown) //最后时刻
		profitt = pow(profitt, this_table.product_type);
	
	// 离4567比较近的时候，提高优先级
	if(distance_bettwen_two(rbt.x, rbt.y, this_table.x, this_table.y) < 5 && this_table.table_type > 3)
	{
		profitt *= 2;
	// 离456比较远，降低优先级，为的是不单独去买456，而是给456送123的时候顺便买
	}else if(distance_bettwen_two(rbt.x, rbt.y, this_table.x, this_table.y) > 5 && this_table.table_type >= 4)
	{
		profitt /= 30;
	}
	
	time = max(time, 1.0); //防止除以0
	buy_tasks.push(q_node(profitt/time, &rbt, &this_table));
}

void sell(Robot &rbt, Table &this_table, priority_queue<q_node>& sell_tasks, vector<vector<double>> &sell_priority)
{
	if(this_table.hava_this_component[rbt.take_type] == true ||
		this_table.want_buy[rbt.take_type] == false)
		return;

	double min_d = closest_buyer(this_table);
	double distance, profitt = 0, time = 0;

	//寻路
	bool have_path = path_finder.run(rbt.x, rbt.y, this_table.x, this_table.y, true, 80);

	if(have_path)
	{
		distance = path_finder.path->cost;
	}else{	//机器人和this_table之间没有路径
		return;
	}
	//卖东西的时间花费
	double 	go_time = distance/speed_expectation*50,
			sell_time = min_d/speed_expectation*50;

	if(this_table.occupation)
		go_time *= 100;
	if(rbt.close_enemy > 0)
		go_time *= 5;

	time = go_time + sell_time;
	profitt = sell_profit[this_table.product_type];

	if(this_table.remain_component == 1)
	{
		profitt *= 2;
	}

	if(sum_time-frameID <= countdown)	//最后时刻只有时间最重要
		time -= sell_time, profitt = 100;

	time = max(time, 1.0); //防止除以0

	sell_tasks.push(q_node(profitt/time, &rbt, &this_table));
	sell_priority[rbt.id][this_table.id] = profitt/time;
}

//做决策,领任务
void decision_making()
{
	priority_queue<q_node> buy_tasks, sell_tasks;
	vector<vector<double>> sell_priority(4, vector<double>(51, -100));
	vector<vector<int>> t_vis(51, vector<int>(10, 0));
	vector<bool> r_vis(4, false);
	vector<int> how_many_table_buy_this(9, 0);

	for(auto &rbt : robots)
	{
		if(rbt.shut_down || rbt.attacker) continue;
		//因为跳帧产生错误的修正
		fix_frameskip(rbt);
		if(rbt.target_table)
		{
			r_vis[rbt.id] = true;
			if(rbt.take_type > 0)
				t_vis[rbt.target_table->id][rbt.take_type] = 1;
			else
				t_vis[rbt.target_table->id][0] = 1;
			continue;
		}
		else
			for(auto &this_table : tables)
			{
				//无效工作台
				if(this_table.shut_down  > 0 || robots_to_tables[rbt.id][this_table.id] == false)
					continue;
				//机器人买
				if(rbt.take_type == 0)
					buy(rbt, this_table, buy_tasks);
				//机器人卖
				else
					sell(rbt, this_table, sell_tasks, sell_priority);
			}
	}
	
	for(auto &t:tables)
	{
		if(t.shut_down) continue;
		for(auto c : the_table_want_buy[t.table_type])
		{
			if(!t.hava_this_component[c])
				++how_many_table_buy_this[c];
		}
	}	

	for(auto &r : robots)
	{
		if(r.shut_down || r.attacker) continue;
		if(r.take_type != 0)
		{
			--how_many_table_buy_this[r.take_type];
			if(r.target_table && r.target_table->remain_component == 1 && r.target_table->remain_time != 0) //这个送完就空了
			{
				for(auto a : the_table_want_buy[r.target_table->table_type])
					++how_many_table_buy_this[a];
			}
		}
		if(r.take_type == 0 && r.target_table)
			--how_many_table_buy_this[r.target_table->product_type];
		
	}

	
	while(!sell_tasks.empty())
	{
		auto a = sell_tasks.top(); sell_tasks.pop();
		auto rbt = a.r;
		auto this_table = a.t;
																// 8,9可以无限卖
		if(r_vis[rbt->id] || (t_vis[this_table->id][rbt->take_type] && this_table->table_type < 8))
			continue;
		
		r_vis[rbt->id] = true;
		t_vis[this_table->id][rbt->take_type] = 1;

		if(sum_time-frameID > countdown && distance_bettwen_two(rbt->x, rbt->y, this_table->x, this_table->y) < 10)
			t_vis[this_table->id][0] = 1;	//如果有人去这个工作台卖，也可以顺便买

		rbt->set_target(this_table);

		for(auto &r:robots)
		{
			if(r_vis[r.id] || no_seven_mode || r.shut_down || r.attacker) continue;

			if(sell_priority[r.id][this_table->id] > 0 && this_table->table_type < 8)
			{
				sell_tasks.push(q_node(sell_priority[r.id][this_table->id] * sell_same_place, &robots[r.id], this_table));
			}
		}
	}
	
	vector<int> balance(10, 0);
	while(!buy_tasks.empty())
	{
		auto a = buy_tasks.top(); buy_tasks.pop();
		auto rbt = a.r;
		auto this_table = a.t;
			//1,2,3生产时间短可以同时买
		if((this_table->table_type > 3 && t_vis[this_table->id][0]) || 
			(this_table->table_type <= 3 && t_vis[this_table->id][0] >= meantime_buy) ||
			r_vis[rbt->id] || 
			balance[this_table->table_type] >= 2 ||
			how_many_table_buy_this[this_table->product_type] <= 0)
			continue;

		++balance[this_table->table_type];
		r_vis[rbt->id] = true, t_vis[this_table->id][0] += 1;
		
		--how_many_table_buy_this[this_table->product_type];

		rbt->set_target(this_table);
	}
}


void init() {
	fgets(line, sizeof line, stdin);
	red = true;
	char r_c = 'B', t_c = 'a'-1, dt_c = '0';
	if(line[0] == 'B')
		red = false, r_c = 'A', t_c = '0', dt_c = 'a'-1;
	if(red)
		++speed_expectation;
	robots[0].id = 0;
	robots[1].id = 1;
	robots[2].id = 2;
	robots[3].id = 3;
	int r_id = 0, t_id = 0, d_t_id = 0;
	for(int y = 99; y >= 0; --y)
	{
		fgets(line, sizeof line, stdin);

		for(int x = 0; x < 100; ++x)
		{
			mp[x][y] = line[x];
			if(line[x] == r_c)
				robots[r_id++].set_state((x+1)*0.5-0.25, (y+1)*0.5-0.25, 0, 0, 0.0, -1, 0);
			else if(line[x] >= t_c+1 && line[x] <= t_c+9) //读到自己的工作台
			{
				int table_type = line[x]-t_c;
				tables.emplace_back(t_id, table_type, table_type<8?table_type:0, the_table_want_buy[table_type].size(), (x+1)*0.5-0.25, (y+1)*0.5-0.25);
														//product_type				//sum_component
				t_id++;
			}else if(line[x] >= dt_c+1 && line[x] <= dt_c+9) { //读到对方的工作台
				int table_type = line[x]-dt_c;
				d_tables.emplace_back(d_t_id, table_type, table_type<8?table_type:0, the_table_want_buy[table_type].size(), (x+1)*0.5-0.25, (y+1)*0.5-0.25);
														//product_type				//sum_component
				d_t_id++;
			}else if(line[x] == '#') {
				++wall_sum;
				path_finder.addObs({x, y, nullptr}, '#');
			}
		}
	}
	fgets(line, sizeof line, stdin); //读“OK”
    return;
}

bool in_corner(Table& t)
{
	int x = (t.x-0.25)*2, y = (t.y-0.25)*2;
	if( (x+1 >= 100 || mp[x+1][y]=='#') + 
		(x-1 < 0 || mp[x-1][y]=='#') + 
		(y+1 >= 100 || mp[x][y+1]=='#') + 
		(y-1 < 0 || mp[x][y-1]=='#')
		 >= 2)
		return true;
	return false;
}

void init_path()
{
	//确定每两个工作台之间的连通性
	for(auto &t_buy : tables) //
	{
		for(auto &t_sell : tables)
		{
			//没有买卖关系
			if(&t_buy == &t_sell || t_buy.want_buy[t_sell.product_type] == false) continue;

			t_buy.shut_down = t_sell.shut_down = 0;
			if(path_finder.run(t_sell.x, t_sell.y, t_buy.x, t_buy.y, true, 0)) //从卖家送到买家手里
			{
				path_bettwen_tables_take[t_sell.id][t_buy.id] = path_finder.path;
				path_bettwen_tables_take[t_buy.id][t_sell.id] = make_shared<Path>();
				path_bettwen_tables_take[t_buy.id][t_sell.id]->cost = path_bettwen_tables_take[t_sell.id][t_buy.id]->cost;
				for(auto p = path_bettwen_tables_take[t_sell.id][t_buy.id]->nodes.crbegin(); p != path_bettwen_tables_take[t_sell.id][t_buy.id]->nodes.crend(); ++p)
					path_bettwen_tables_take[t_buy.id][t_sell.id]->nodes.push_back(*p);
			}

		}
	}
	
	//确定机器人和工作台之间的连通性
	for(auto &r : robots)
	{
		for(auto &t : tables)
		{
			//在角落的456789
			if(t.shut_down > 0 || (in_corner(t) && t.table_type > 3))
				continue;
			if(path_finder.run(r.x, r.y, t.x, t.y, false, 0))
			{
				r.shut_down = 0;
				robots_to_tables[r.id][t.id] = true;
			}
		}
	}
}

void chose_attacker()
{
	bool flag = true;
	for(auto &r : robots)
		if(r.shut_down)
		{
			r.attacker = true;
			flag = false;
		}
	if(flag && d_tables.size() > 0 && wall_sum >= 2000)
		robots[0].attacker = true;
}

void chose_attack_table()
{
	// vector<pair<double, Table*>> ts(0);
	// for(auto &dt : d_tables) //所有23456789工作台
	// {
	// 	int x = (dt.x-0.25)*2, y = (dt.y-0.25)*2, num = 0;
	// 	int s_x = x-3 < 0 ? 0 : x-3, s_y = y-5 < 0 ? 0 : y-3,
	// 		e_x = x+3 > 49 ? 49 : x+3, e_y = y+5 > 49 ? 49 : y+3;
	// 	for(int i = s_x; i <= e_x; ++i)
	// 	{
	// 		for(int j = s_y; j <= e_y; ++j)
	// 		{
	// 			if(path_finder.grid[i][j] == '#')
	// 				++num;
	// 		}
	// 	}
	// 	ts.emplace_back((double)num, &dt);
	// }
	// sort(ts.begin(), ts.end());
	// for(auto &p : ts)
	// {
	// 	block_list.emplace_back(p.second);
	// 	if(block_list.size() > 4)
	// 		break;
	// }

	vector<pair<double, Table*>> ts(0);
	for(auto &dt : d_tables) //所有23456789工作台
	{
		if(dt.table_type < 3) continue;
		double d_sum = 0; //距离456工作台的总距离
		int num = 0;
		for(auto &t4567 : d_tables)
		{
			if(&dt == &t4567) continue;
			if(t4567.table_type < 4 || t4567.table_type > 6) continue;
			// if(dt.want_buy[t4567.product_type] == false)	continue;
			if(path_finder.run(t4567.x, t4567.y, dt.x, dt.y, true, 80))
			{
				d_sum += path_finder.path->cost;
				++num;
			}
		}
		if(num > 0)
		{
			ts.emplace_back(d_sum/num, &dt);
		}
	}
	sort(ts.begin(), ts.end());
	for(auto &p : ts)
	{
		block_list.emplace_back(p.second);
		if(block_list.size() > 4)
			break;
	}
}

void read_tables()
{
	int k; scanf("%d\n", &k);
	int type, remaiin_time, component_state, have_product;
	double x, y;
	int id = 0;

	//更新工作台信息
	for(int id = 0; id < k; ++id)
	{
		scanf("%d %lf %lf %d %d %d\n", &type, &x, &y, &remaiin_time, 
								&component_state, &have_product);
		if(tables[id].shut_down > 0)
		{
			--tables[id].shut_down;
			continue;
		}
		if(tables[id].occupation > 0)
			--tables[id].occupation;
		tables[id].set_state(remaiin_time, component_state, have_product);
	}
}

//得到敌方机器人的位置
void get_enemy_pos(vector<Radar_line> &radar)
{
	for(int i = 0; i <= radar.size()-5; i+=5)
	{
		double x0 = radar[i].x,   y0 = radar[i].y, d0 = radar[i].d, 
			//    x1 = radar[i+1].x, y1 = radar[i+1].y,
			   x2 = radar[i+2].x, y2 = radar[i+2].y, d2 = radar[i+2].d, 
			//    x3 = radar[i+3].x, y3 = radar[i+3].y,
			   x4 = radar[i+4].x, y4 = radar[i+4].y, d4 = radar[i+4].d;
		double x = 0, y = 0, r = 0, xx = 0 , yy = 0, rr = 0;
		if(tree_point_circular(x0, y0, x2, y2, x4, y4, r, x, y))
		{
			bool flag = true;
			for(auto &rbt : robots)
			{
				if( flag && distance_bettwen_two(rbt.x, rbt.y, x, y) < 0.1) //说明是友军
				{
					flag = false;
					break;
				}
			}
			for(auto &e : enemys)
			{
				if(flag && distance_bettwen_two(e.x, e.y, x, y) < 0.01) //说明已经加入过了
				{
					flag = false;
					break;
				}
			}
			if(flag)
			{
				enemys.emplace_back(x,y,r);
				continue;
			}
		}
	}
}

//扫描左右range度的范围，一共2*range度
void read_radar(int range)
{
	enemys.clear();
	vector<Radar_line> radar(2*range, {0,0,0});
	for(auto &r : robots)
	{
		int cnt = 360;
		double d;
		for(int a = 0; a < 360; ++a)
		{
			scanf("%lf\n", &d);
			r.rdar[a] = d;
			if(a <= range-1) // 0-range-1 度的情况
			{
				double ag = (r.face + a/180.0*M_PI) > M_PI ? (r.face + a/180.0*M_PI) - 2*M_PI : (r.face + a/180.0*M_PI);
				double x = r.x + cos(ag)*d, y = r.y + sin(ag)*d;
				radar[a + range] = {x,y,d}; //填入radar[range ~ 2*range-1]
			}else if(a >= 360-range) // 360-range ~ 359度的情况
			{
				double re_a = a - 360; //-4 ~ -1
				double ag = (r.face + re_a/180.0*M_PI) < -M_PI ? (r.face + re_a/180.0*M_PI) + 2*M_PI : (r.face + re_a/180.0*M_PI);
				double x = r.x + cos(ag)*d, y = r.y + sin(ag)*d;
				radar[a - 360 + range] = {x,y,d};//填入radar[0 ~ range-1]
			}
		}
		get_enemy_pos(radar);

	}
}

void read_robots()
{

	for(auto &r :robots)
	{
		int table_id, take_type;
		double time, will_crash, ro, lx, ly, face, x, y;
		scanf("%d %d %lf %lf %lf %lf %lf %lf %lf %lf\n",
				&table_id, &take_type, &time, &will_crash, &ro, &lx, &ly, &face, &x, &y);
		r.set_state(x, y, lx, ly, face, table_id, take_type);

		//进行交易
		if(!r.attacker)
			r.make_deal();
	}

	read_radar(175);
}

void update_priority(){
	sell_profit = buy_profit = profit_back;
	vector<int> thing_num(8, 0);
	for(auto &t : tables)
	{
		if(t.shut_down || t.occupation > 0) continue;
		if(t.table_type >3)	
		{
			//统计4567产品槽里物品的数量
			if(t.have_product)
				++thing_num[t.product_type];
			else if(t.have_product && t.remain_time == 0)
				++thing_num[t.product_type];
			else if(t.remain_time > 0)
				++thing_num[t.product_type];
			
			//统计4567配件槽里物品的数量
			for(int x : {1,2,3,4,5,6})
			{
				if(t.hava_this_component[x])
					++thing_num[x];
			}
		}
			
	}

	for(auto &r : robots)
	{
		if(r.shut_down > 0 || r.attacker) continue;
		if(r.take_type > 0)
			++thing_num[r.take_type];
		else if(r.take_type == 0 && r.target_table && r.target_table->table_type < 4)
			++thing_num[r.target_table->table_type];
	}

	for(int x : {4,5,6})
	{
		if(thing_num[x] == 0)	//场上没有x，那就把配件卖给x
			sell_profit[x] += profit_back[x];
	}
	
	thing_num[1] += thing_num[4] + thing_num[5];
	thing_num[2] += thing_num[4] + thing_num[6];
	thing_num[3] += thing_num[5] + thing_num[6];


	for(int x : {1,2,3})
	for(int y : {1,2,3})
		if(thing_num[x]< thing_num[y])
			buy_profit[x] += profit_back[x]/3;
}

void set_obs()
{
	for(auto &r : robots)
	{
		int x = (r.x-0.25)*2, y = (r.y-0.25)*2;
		if(r.lock_time > 0 || r.attacker)
			path_finder.addObs({x,y,nullptr}, 80);
		else
			path_finder.addObs({x,y,nullptr}, (char)10*r.take_type + r.id);
	}

	// for(auto &e : enemys)
	// {
	// 	for(auto &t : tables)
	// 	{
	// 		if(distance_bettwen_two(t.x, t.y, e.x, e.y) >= 1.2)
	// 		{
	// 			int x = (e.x-0.25)*2, y = (e.y-0.25)*2;
	// 			path_finder.addObs({x,y,nullptr}, 'x');
	// 		}
	// 	}
	// }
}

void clear_obs()
{
	for(auto &r : robots)
	{
		int x = (r.x-0.25)*2, y = (r.y-0.25)*2;
		path_finder.subObs({x,y,nullptr}, (char)10*r.take_type + r.id);
	}
	// for(auto &e : enemys)
	// {
	// 	for(auto &t : tables)
	// 	{
	// 		if(distance_bettwen_two(t.x, t.y, e.x, e.y) >= 1.2)
	// 		{
	// 			int x = (e.x-0.25)*2, y = (e.y-0.25)*2;
	// 			path_finder.subObs({x,y,nullptr}, 'x');
	// 		}
	// 	}
	// }
}

void out_put()
{
	for(auto &r : robots)
	{
		if(robots_speed[r.id][0] > -99)
			printf("forward %d %f\n", r.id, robots_speed[r.id][0]);
		if(robots_speed[r.id][1] > -99)
			printf("rotate %d %f\n", r.id, robots_speed[r.id][1]);
	}


	fgets(line, sizeof line, stdin);
	printf("OK\n");
	fflush(stdout);
}

void make_route()
{
	//更新正常机器人的路线
	for(auto &r : robots)
	{
		if(r.attacker) continue;
		if((r.slow_speed >= 30 && (r.close_enemy + r.close_my_robot > 0)) || r.slow_speed >= 100)
		{
			if(r.lock_time == 0)
				r.lock_time = 200;
		}

		if(r.target_table && r.lock_time == 0 && r.shut_down == 0)
		{
			r.update_path();
			
		}
		
		if(r.lock_time)
		{
			// int d = 0;
			if(r.close_enemy) //被对方卡死
			{
				
				r.clear();
				r.shut_down = 2;

				// 全速转
				robots_speed[r.id][1] = M_PI/3;
				robots_speed[r.id][0] = max_lspeed;

				// //开始动了，不转，开始撞
				if(distance_bettwen_two(r.lx, r.ly , 0, 0) > 0.1)
				{
					// d = 2;
					robots_speed[r.id][1] = M_PI/10;
					robots_speed[r.id][0] = max_lspeed;
				}
			}else{
				// d = 4;
				// d = 2;
				robots_speed[r.id][0] = -2;
				robots_speed[r.id][1] = M_PI/3;
			}
		} else if(r.shut_down || r.target_table == nullptr) {
			r.avoid();
		} else if(r.path) {
			r.follow_path();
			r.avoid_crash();
		}
	}


	for(auto &r : robots)
	{
		if(!r.attacker) continue;
		if(r.target_table)
		{
			if(distance_bettwen_two(r.x, r.y, r.target_table->x, r.target_table->y) < 0.3)
			{
				robots_speed[r.id][0] = 0;
				robots_speed[r.id][1] = 0;
			}
			else{
				r.update_path();
				r.follow_path();
			}
			
		}else if(r.tar_x > 0)
		{
			r.go(r.tar_x, r.tar_y);
		}
	}
}

void attack_target()
{
	for(auto &r : robots)
	{
		if(!r.attacker) continue;
		// r.attacking = false;
		r.tar_x = r.tar_y = -1;
		double min_d = 1e9;
		for(auto &e : enemys)
		{
			double d = distance_bettwen_two(r.x, r.y, e.x, e.y), ag = abs(angle(r.face, atan2(e.y-r.y,e.x-r.x))); 
			// double dd = red ? 8 : 3;
			if(d < 4 && d + ag < min_d)
			{
				min_d = d + ag;
				r.tar_x = e.x, r.tar_y = e.y;
				// r.attacking = true;
				r.target_table = nullptr;
			}
		}

		bool flag = true;
		for(auto &t : tables)
		{
			if(r.tar_x > 0 && distance_bettwen_two(r.tar_x, r.tar_y, t.x, t.y) < 2) //对方机器人离自己工作台太近，取消攻击
			{
				flag = false;
				break;
			}
		}

		if(flag == false || r.tar_x < 0)
		{
			auto t = block_list[((r.id)%2 + frameID/500)%block_list.size()];
			r.target_table = t;
			r.tar_x = t->x, r.tar_y = t->y;
		}
	}
}

void update_enemy()
{
	for(auto &t : tables)
	{
		if(t.shut_down || t.occupation) continue;
		for(auto &e : enemys)
		{
			if(distance_bettwen_two(t.x, t.y, e.x, e.y) < 1)
			{
				t.occupation = 250;
				// for(auto &r : robots)
				// {
				// 	if(r.target_table == &t)
				// 		r.clear();
				// }
			}
				
		}
	}
}

void set_hit_pos()
{
	for(auto &t : tables)
	{
		if(t.shut_down) continue;
		for(int i : {0,2,-2})
		{
			for(int j : {0,2,-2})
			{
				if(path_finder.can_zip((t.x-0.25)*2, (t.y-0.25)*2, (t.x+i-0.25)*2, (t.y+j-0.25)*2))
				// if(path_finder.run(t.x, t.y, t.x+i, t.y+j, true, 80))
					t.hit_x = t.x+i, t.hit_y = t.y+j;
			}
		}
	}
}

int main() {
	// ofstream fout("in.txt");
	// streambuf* pOld = cerr.rdbuf(fout.rdbuf());
    init();
	init_path();
	set_hit_pos();

	chose_attacker();

	chose_attack_table();

    puts("OK");
    fflush(stdout);
	
    while (scanf("%d %d\n", &frameID, &money) != EOF) {
        printf("%d\n", frameID);

		//更新工作台信息
		read_tables();
		
		//更新机器人信息
		read_robots();

		update_enemy();

		//将机器人设为障碍物
		set_obs();

		//更新工作台的优先级
		update_priority();

		//为工作机器人做决策
		decision_making();

		attack_target();

		make_route();

		//清除机器人障碍物
		clear_obs();
		
		//输出
		out_put();
    }
    return 0;
}
