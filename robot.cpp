#include <algorithm>
#include <iostream>
#include <memory>
#include <cmath>
#include "robot.hh"
#include "toll.hh"
#define M_PI		3.14159265358979323846
vector<Robot> robots(4);
// void debug_robot(const Robot& r);


Robot::Robot () :face(0), target_table(nullptr), lx(0), ly(0), task_type(-1), shut_down(INT32_MAX/2), 
			attacker(false), slow_speed(0), close_my_robot(0), close_enemy(0), lock_time(0), rdar(360, 0) {}

pair<double, double> Robot::int_to_double(const pair<int, int>& n)
{
	int int_x = n.first, int_y = n.second;
	double x = int_x/2.0+0.25, y = int_y/2.0+0.25;

	if(path_finder.blocked(int_x, int_y+1))
		y -= 0.25;
	else if(path_finder.blocked(int_x, int_y-1))
		y += 0.25;
	
	if(path_finder.blocked(int_x+1, int_y))
		x -= 0.25;
	else if(path_finder.blocked(int_x-1, int_y))
		x += 0.25;
	// if(int_y+1 > path_finder.yh || path_finder.grid[int_x][int_y+1] == '#')
	// 	y -= 0.25;
	// else if(int_y-1 < path_finder.yl || path_finder.grid[int_x][int_y-1]  == '#')
	// 	y += 0.25;
	// if(int_x+1 > path_finder.xh || path_finder.grid[int_x+1][int_y]  == '#')
	// 	x -= 0.25;
	// else if(int_x-1 < path_finder.xl || path_finder.grid[int_x-1][int_y]  == '#')
	// 	x += 0.25;
	
	return {x,y};
}

//设置当前状态
void Robot::set_state(double x, double y, double lx, double ly, double face, int in_table, int take_type)
{
	this->x = x;
	this->y = y;
	this->lx = lx;
	this->ly = ly;
	this->in_table_id = in_table;
	this->take_type = take_type;
	this->face = face;
	robots_speed[id][0] = robots_speed[id][1] = -100;


	double min_d = 1e9;
	Enemy *e_p = nullptr;
	for(auto &e : enemys)
	{
		if( e_p == nullptr || distance_bettwen_two(e.x, e.y, x, y)  < min_d)
		{
			e_p = &e;
			min_d = distance_bettwen_two(e.x, e.y, x, y);
		}
	}
	// 被enemy推
	bool slow = false;
	if(e_p && min_d < 1.1)
	{
		if(angle(face, atan2(ly,lx)) > 4*M_PI/5)
		{
			++slow_speed; 
			slow = true;
		}
	}
	double speed = distance_bettwen_two(lx, ly, 0, 0);
	if(speed < 0.01)
	{
		++slow_speed;
		slow = true;
	}
	
	if(!slow && slow_speed > 0)
		--slow_speed;

	if(speed > 1.5)
		lock_time = 0;
		
	close_enemy = close_my_robot = 0;
	for(auto &r : robots)
	{
		if(&r == this)	continue;
		if(distance_bettwen_two(x,y, r.x, r.y) < 1.1)
			++close_my_robot;
	}

	for(auto &e : enemys)
	{
		if(distance_bettwen_two(x,y, e.x, e.y) < 1.1)
			++close_enemy;
	}

	if(shut_down > 0)
		--shut_down;
	if(lock_time > 0)
		--lock_time;
	
	++frame_num_this_target;
	if(!attacker && target_table && frameID%50==0 && (frame_num_this_target > 150 || (target_table->occupation > 0)))
	{
		clear();
	}
	if(!attacker && target_table && target_table->occupation>0 && hit_pos == false && distance_bettwen_two(x,y,target_table->hit_x, target_table->hit_y) < 1.5)
	{
		hit_pos = true;
	}
}

//朝目标（dx, dy）出发
void Robot::go(double dx, double dy)
{
	double angleSpeed = 0;
	double lineSpeed = max_lspeed;
		
	double distance = distance_bettwen_two(x,y,dx,dy);
	lineSpeed = distance*3;
	lineSpeed = min(lineSpeed, max_lspeed);

	double destination_dir = atan2(dy-y,dx-x);
	double ag = angle(face, destination_dir); //得到正转和逆转的所需角度
	angleSpeed = max_rspeed*ag/(M_PI/6);
	lineSpeed *= (M_PI/2 - abs(ag));
	if(abs(ag) >= M_PI/2) //钝角
		stay();
	else if(abs(ag) >= M_PI/4)
		lineSpeed = 1;

	if(angleSpeed > M_PI)
		angleSpeed = M_PI;
	else if(angleSpeed < -M_PI)
		angleSpeed = -M_PI;

	lineSpeed = min(lineSpeed, max_lspeed);
	//同目的地避免碰撞
	for(auto &r : robots)
	{
		if(&r == this || target_table==nullptr || r.attacker) //
			continue;

		double r_d = distance_bettwen_two(r.y, r.x, target_table->y, target_table->x);
		if(r_d < 0.5 && distance_bettwen_two(x,y,target_table->x,target_table->y) < 3)	//目的地有人
			lineSpeed /= slow_down;
	}			

	if( target_table &&  target_table->x == dx && target_table->y == dy && target_table->occupation)
	{
		if((distance_bettwen_two(x, y, dx, dy) >= 4 && ag < M_PI/16))
			lineSpeed = max_lspeed;
		else if(distance_bettwen_two(x, y, dx, dy) < 2 && distance_bettwen_two(lx, ly, 0,0) < 0.5)
		{
			lineSpeed = -2;
		}

	}

	if(robots_speed[id][0] < -99)
		robots_speed[id][0] = lineSpeed;
	if(robots_speed[id][1] < -99)
		robots_speed[id][1] = angleSpeed; 
	
}

void Robot::attack(double dx, double dy)
{
	double destination_dir = atan2(dy-y,dx-x);
	double ag = angle(face, destination_dir); //得到正转和逆转的所需角度
	double angleSpeed = max_rspeed*ag/(M_PI/6), lineSpeed = abs(ag) > M_PI/8 ? 0 : max_lspeed;

	if(angleSpeed > M_PI)
		angleSpeed = M_PI;
	else if(angleSpeed < -M_PI)
		angleSpeed = -M_PI;
	
	robots_speed[id][0] = lineSpeed;
	robots_speed[id][1] = angleSpeed;
}


void Robot::follow_path()
{
			
	if(path == nullptr)
	{
		return;
	}
	
	double x, y;
	while(follow_indx <= path->nodes.size())
	{
		if(follow_indx >= path->nodes.size())
		{
			auto p_double = int_to_double({path->nodes.back()->x, path->nodes.back()->y});
			x = p_double.first, y = p_double.second;
			break;
		}
		auto p_double = int_to_double({path->nodes[follow_indx]->x, path->nodes[follow_indx]->y});
		x = p_double.first, y = p_double.second;
		double close = (follow_indx == path->nodes.size()-1 ? 0.4 : 0.6);

		// if(attacker)
		// 	close = 0.7;
		if( distance_bettwen_two(x, y, this->x, this->y) < close)
		{
			++follow_indx;
		}
		else
			break;
	}
	
	go(x, y);	
}

void Robot::avoid_crash()
{
	if(distance_bettwen_two(lx, ly, 0, 0) > 1)
	{
		for(auto &r : robots)
		{
			if(this == &r || r.take_type < take_type || distance_bettwen_two(x, y, r.x ,r.y) > 5) continue;
			if(r.take_type == 0 && take_type == 0) continue;
			double 	position_angle = angle(atan2(ly, lx), atan2(r.y-y,r.x-x)), 
					crash_angle = asin(diameter_robot_with/ distance_bettwen_two(r.y, r.x, y, x));
			if(abs(position_angle) < crash_angle)
			{
				if(robots_speed[id][0] > -99)
					robots_speed[id][0]  /= 2;
				return;
			}
		}
	}
	

	// 距离目标地比较远再破死锁(蓝色直接撞)
	// if(red && target_table && distance_bettwen_two(x, y, target_table->x, target_table->y) > 2)
	// 	for(auto &e : enemys)
	// 	{
	// 		if(distance_bettwen_two(x, y, e.x ,e.y) > 2) continue;

	// 		double 	position_angle = angle(face, atan2(e.y-y,e.x-x)); 
	// 				// crash_angle = asin(diameter_robot_with/ distance_bettwen_two(e.y, e.x, y, x));
	// 		if(abs(position_angle) < M_PI/2)
	// 		{
	// 			robots_speed[id][0]  = max_lspeed;
	// 			robots_speed[id][1]  = (position_angle > 0 ? -1 : 1) * M_PI/5;
	// 			return;
	// 		}
	// 	}

}


void Robot::avoid()
{	
	// double ddx, ddy;
	// path_finder.step = 100;
	// for(int i : {0,1,-1})
	// {
	// 	for(int j : {0,1,-1})
	// 	{
	// 		double dx = x+i, dy = y+j;
			
	// 		if(path_finder.blocked((dx-0.25)*2, (dy-0.25)*2))
	// 			continue;
			
	// 		bool flag = true;
	// 		for(auto &r : robots)
	// 		{
	// 			if(!flag || this == &r || r.attacker || r.path == nullptr || distance_bettwen_two(x, y, r.x, r.y) > 10) continue;
	// 			auto &n = r.path->nodes;
	// 			for(int i = 0; i < n.size()-1; ++i)
	// 			{
	// 				double start_x = (n[i]->x/2.0+0.25), start_y = (n[i]->y/2.0+0.25), end_x = (n[i+1]->x/2.0+0.25), end_y = (n[i+1]->y/2.0+0.25);
	// 				double d = point_to_line_segment_distance(dx, dy, start_x, start_y, end_x, end_y);
	// 				if(d < 1.5)
	// 				{
	// 					flag = false;
	// 					break;
	// 				}
	// 			}
				
	// 		}
	// 		if(flag)
	// 		{
	// 			go(dx, dy);
	// 			return;
	// 		}
	// 	}
	// }
	Robot *rp = nullptr;
	double d = 0;
	for(auto &r : robots)
	{
		if(this == &r || distance_bettwen_two(x, y, r.x , r.y) > 5) continue;
		if(rp == nullptr)
		{
			rp = &r;
			d = distance_bettwen_two(x, y, r.x, r.y);
		}
		else if(distance_bettwen_two(x, y, r.x, r.y) <  d)
		{
			d = distance_bettwen_two(x, y, r.x, r.y);
			rp = &r;
		}
	}

	if(rp)
	{
		double dx = x + (x - rp->x), dy = y + (y - rp->y);
		if(dx < 0) dx = 0;
		else if(dx > 50) dx = 50;
		if(dy < 0) dy = 0;
		else if(dy > 50) dy = 50;
		
		go(dx, dy);
		return;
	}

	robots_speed[id][0] = -2;
	// robots_speed[id][1] = M_PI;
}

//停住
void Robot::stay()
{
	double ag = abs(angle(atan2(ly, lx), face));
	if(ag < M_PI/2) //同向
		robots_speed[id][0] = -1*distance_bettwen_two(lx, ly, 0, 0);
	else
		robots_speed[id][0] = distance_bettwen_two(lx, ly, 0, 0);
	// robots_speed[id][1] = 0;
		
		
	// printf("forward %d %d\n", id, 0);
	// printf("rotate %d %f\n", id, 0);
}

void Robot::clear()
{
	cannot_find_path_num = 0;
	target_table = nullptr;
	task_type = -1;
	follow_indx = 1;
	frame_num_this_target = 0;
	path = nullptr;
}

void Robot::set_path(shared_ptr<Path>& pt)
{
	path = pt;
	follow_indx = 1;
	cannot_find_path_num = 0;
}

void Robot::set_target(Table* t)
{
	target_table = t;
	if(t->occupation > 0)
		hit_pos = false;
	frame_num_this_target = 0;
	task_type = (take_type > 0);
}

void Robot::make_deal()
{
	if(target_table != nullptr && in_table_id == target_table->id) {
		if(target_table->want_buy[take_type] && target_table->hava_this_component[take_type] == false)
		{
			stay();
			printf("sell %d\n", id);
			target_table->hava_this_component[take_type] = true;
			take_type = 0;
		}else if(take_type == 0 && target_table->have_product)
		{
			stay();
			printf("buy %d\n", id);
			take_type = target_table->product_type;
			target_table->have_product = 0;
		}
		clear();
	}
}

void Robot::update_path()
{
	
	bool have_path;

	if(attacker && tar_x > 0 && tar_y > 0)
	{
		have_path = path_finder.run(x, y, tar_x, tar_y, 0, 'x');
	}
	else if(!attacker)
	{
		if(target_table->occupation > 0 && hit_pos == false)
			have_path = path_finder.run(x, y, target_table->hit_x, target_table->hit_y, take_type>0, 10*take_type + id);
		else
			have_path = path_finder.run(x, y, target_table->x, target_table->y, take_type>0, 10*take_type + id);

	}

	if(have_path)
	{
		set_path(path_finder.path);
	}else{	//机器人和this_table之间没有路径
		path = nullptr;
		// if(!attacker);
			// shut_down = 1;
	}
}

bool Robot::face_empty(int ag, double d)
{
	for(int i = 0; i < ag; ++i)
	{
		if(rdar[i] < d || rdar[359-i] < d)
			return false;
	}
	return true;
}

int Robot::empty_ag(double d)
{
	int res = 0, sum = 0;
	for(auto &a : rdar)
	{
		if(a > d)
			++sum;
		else {
			res = max(res, sum);
			sum = 0;
		}
	}
	res = max(res, sum);
	return res;
}

