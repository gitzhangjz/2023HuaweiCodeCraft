#ifndef ROBOT_HH
#define ROBOT_HH
#include "table.hh"
#include "arg.hh"
#include "toll.hh"
#include "A_star.hh"

struct Robot {
	int id,		//机器人id
		in_table_id,	//当前所在桌子id
		take_type,		//拿到的物品类型
		follow_indx,
		cannot_find_path_num,
		frame_num_this_target;
	Table	*target_table;//目标桌子的id
	double 	face,		//面向角度
			x,			//坐标x
			y,			//坐标y
			lx,
			ly,
			tar_x,
			tar_y;
	int shut_down, lock_time
		,slow_speed
		,close_my_robot
		,close_enemy;
		;
	shared_ptr<Path> path;
	int task_type;
	bool attacker, hit_pos;
	vector<double> rdar;
	Robot ();


	// 进行交易
	void make_deal();

	//设置当前状态
	void set_state(double x, double y, double lspeed_x, double lspeed_y, double face, int table_id, int take_type);

	//朝目标（dx, dy）出发
	void go(double dx, double dy);

	// 面前是否是空的
	bool face_empty(int , double);

	//周围射线距离大于d的角度数
	int empty_ag(double d);

	// 攻击这个位置
	void attack(double dx, double dy);

	//已经有path，跟着path走
	void follow_path();

	// 避免碰撞
	void avoid_crash();

	//停住
	void stay();

	// 避免挡其他机器人（让路状态）
	void avoid();

	// 清空当前任务和路径
	void clear();

	//设置路径
	void set_path(shared_ptr<Path>& p);

	// 设置目标
	void set_target(Table* t);

	// 更新路径
	void update_path();

	pair<double, double> int_to_double(const pair<int, int>& n);
};
extern vector<Robot> robots;

#endif