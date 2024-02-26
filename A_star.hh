#ifndef A_STAR_HH
#define A_STAR_HH

#include <vector>
#include <memory>

using namespace std;

struct Node
{
	//坐标
	int x, y, step;
	shared_ptr<Node> father;
	//Node构造函数
	Node(int x, int y, shared_ptr<Node> f, int step = 0);
	Node();
};
typedef	 shared_ptr<Node> NodePtr;

struct Path
{
	//Path数据
	vector<NodePtr> nodes;

	//Path代价
	double cost;

	//构造函数
	Path();

	//向路径添加Node
	void addNode(const NodePtr&);
};

struct A_star
{
	//图的边界
	int xl, yl, xh, yh, start_x, start_y, end_x, end_y, step;
	char r_tag;
	//图的数据
	vector<vector<char>> grid;

	shared_ptr<Path> path;
	shared_ptr<Path> short_path;

	//构造函数
	A_star();

	//向地图添加障碍物（包括机器人）
	void addObs(const Node&, char c);

	//清除障碍物（机器人）
	void subObs(const Node&, char c);

	// 从（x,y） 向 (x+d_x, y+d_y) 移动是否可行
	bool islegal(int x, int y, int d_x, int d_y, bool, bool);

	// 移动到当前位置的cost
	double cost(int x, int y);

	//搜索路径，take 表示是否带了货物，r_tag表示这个机器人的优先级
	bool run(double start_x, double start_y, double end_x, double end_y, bool take, char r_tag);

	//建立路径
	void build_path(NodePtr);

	// 压缩路径
	void zip_path();

	// 是否越界
	bool crossed(int dx, int dy);

	// 是否被挡
	bool blocked(int dx, int dy);

	// 这个障碍是否是机器人
	bool is_robot(int dx, int dy);

	// 寻找路径中下一个可压缩节点（二分查找，和元龙哥想到一起了哈哈哈）
	int next_zip_node(int target);

	// node[indx1] 和 node[indx2] 是否可以直达（中间没有障碍物）
	bool can_zip(int indx1, int indx2);
	bool can_zip(int x0, int y0, int x1, int y1);
};

extern A_star path_finder;

#endif