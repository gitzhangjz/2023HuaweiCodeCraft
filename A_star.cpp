#include "A_star.hh"
#include "toll.hh"
#include "arg.hh"
#include <vector>
#include <cmath>
#include <queue>
#include <iostream>
using namespace std;
A_star path_finder;


bool cmp (const pair<NodePtr, double>& a, const pair<NodePtr, double>& b) {
	return a.second > b.second;
}


Node::Node(int x, int y, shared_ptr<Node> father, int step) : x(x), y(y), father(father), step(step) {}
Node::Node() {}

Path::Path() : cost(0), nodes(0) {}

void Path::addNode(const NodePtr& n){
	nodes.emplace_back(n);
}

A_star::A_star() : grid(100, vector<char>(100, '.')), xl(0), yl(0), xh(99), yh(99), path(nullptr), short_path(nullptr) {}

bool A_star::crossed(int dx, int dy)
{
	return dx < xl || dx > xh || dy < yl || dy > yh;
		
}

bool A_star::blocked(int dx, int dy)
{
	if(r_tag == 'x')
	{
		return crossed(dx,dy) || grid[dx][dy] == '#';
	}

	if(crossed(dx,dy) || grid[dx][dy] == '#')
		return true;
	if(grid[dx][dy] == '.')
		return false;
	if(step < 25 && grid[dx][dy] > r_tag)
	{
		return true;
	}
	return false;
}

bool A_star::is_robot(int dx, int dy)
{
	if(grid[dx][dy] != '.' && grid[dx][dy] != '#' && grid[dx][dy] != r_tag)
		return true;
	return false;
}

void A_star::addObs(const Node &n, char c)
{
	if(c == '#')
		grid[n.x][n.y] = c;
	else//机器人作为障碍
	{
		//把机器人四周都当作障碍
		for(auto &d : dir)
		{
			if(d[0] != 0 && d[1] != 0)
				continue;
			int dx = n.x+d[0], dy = n.y+d[1];
			if(!crossed(dx, dy) && grid[dx][dy] != '#')
				grid[dx][dy] = c;
		}
	}
}
void A_star::subObs(const Node&n, char c)
{
	//把机器人四周都清除
	for(auto &d : dir)
	{
		if(d[0] != 0 && d[1] != 0)
				continue;
		int dx = n.x+d[0], dy = n.y+d[1];
		if(!crossed(dx, dy) && (grid[dx][dy] != '#'))
			grid[dx][dy] = '.';
	}

}

double A_star::cost(int x, int y)
{
	int wall_num = 0, rbt_num = 0;
	for(auto &d : dir)
	{
		int dx = d[0]+x, dy = d[1]+y;
		if(blocked(dx, dy))
			++wall_num;
		if(!crossed(dx, dy) && is_robot(dx, dy))
			++rbt_num;
	}
	
	return min(4, rbt_num) + min(wall_num, 2)*2 + distance_bettwen_two(x, y, start_x, start_y) + 
						  distance_bettwen_two(x, y, end_x, end_y);
}


bool A_star::islegal(int s_x, int s_y, int d_x, int d_y, bool isterminal, bool take)
{
	
	int x = s_x+d_x, y = s_y+d_y;
	if(blocked(x,y))
		return false;
	
	//走45度
	if(d_x != 0 && d_y != 0)
	{
		if(blocked(x, s_y) || blocked(s_x, y))
			return false;
	}

	//两对面有墙
	if( (blocked(x+1, y) && blocked(x-1,y)) || (blocked(x, y+1) && blocked(x, y-1)) )
		return false;
	//两斜对角有墙
	if( (blocked(x+1, y+1) && blocked(x-1,y-1)) || (blocked(x-1, y+1) && blocked(x+1, y-1)) )
		return false;
	//夹角墙，拿着物品
	if((blocked(x+1, y) + blocked(x-1,y) + blocked(x,y+1) + blocked(x,y-1) >= 2) && take)
		return false;
	//拿着物品，一面有墙就死
	if(!isterminal && take && (blocked(x+1, y) || blocked(x-1,y) || blocked(x, y+1) || blocked(x,y-1)))
		return false;
	if(take && d_x != 0 && d_y != 0)
	{
		if(blocked(s_x+2*d_x, s_y) || blocked(s_x, s_y+2*d_y))
			return false;
	}
	if(d_x != 0 && d_y == 0)
	{
		if(blocked(s_x,s_y+1)&&blocked(s_x+d_x, s_y-1))
			return false;
		else if(blocked(s_x,s_y-1)&&blocked(s_x+d_x, s_y+1))
			return false;
	}
	if(d_x == 0 && d_y != 0)
	{
		if(blocked(s_x+1,s_y)&&blocked(s_x-1, s_y+d_y))
			return false;
		else if(blocked(s_x-1,s_y)&&blocked(s_x+1, s_y+d_y))
			return false;
	}
	
	return true;
}

bool A_star::run(double x1, double y1, double x2, double y2, bool take, char r_tag)
{
	int start_x = (x1-0.25)*2, start_y = (y1-0.25)*2, end_x = (x2-0.25)*2, end_y = (y2-0.25)*2;
	this->r_tag = r_tag;
	this->start_x = start_x;
	this->start_y = start_y;
	this->end_x = end_x;
	this->end_y = end_y;
	this->step = 0;

	// if(blocked(end_x, end_y))
	// {
	// 	if(frameID == 134)
	// 	return false;
	// }

	// if(start_x == end_x && start_y == end_y)
	// 	return true;

	priority_queue<pair<NodePtr, double>, vector<pair<NodePtr, double>>, decltype(&cmp)> q(cmp);
	vector<vector<bool>> close_set(xh+1, vector<bool>(xh+1, false));
	vector<vector<bool>> open_set(xh+1, vector<bool>(xh+1, false));
	//起点加入队列
	q.push({make_shared<Node>(start_x, start_y, nullptr, 0), 0});
	open_set[start_x][start_y] = true;

	while (!q.empty())
	{
		auto n = q.top().first;  q.pop();

		int x = n->x, y = n->y;
		step = n->step;
		
		if(end_x == x && end_y == y) //搜到了
		{
			build_path(n);
			return true;
		}
		close_set[x][y] = true;
		open_set[x][y] = false;
		for(auto &d : dir)
		{
			int dx = d[0]+x, dy = d[1]+y;
			if(crossed(dx, dy) || close_set[dx][dy] || open_set[dx][dy])
				continue;
			
			if(islegal(x, y, d[0], d[1], dx==end_x && dy==end_y, take))
			{
				auto new_node = make_shared<Node>(dx, dy, n, step+1);
				
				q.push({new_node, cost(dx, dy)});
				open_set[dx][dy] = true;
			}
		}
	}
	
	return false;
}


void A_star::build_path(NodePtr n)
{
	step = 0;
	path = make_shared<Path>();
	while(n)
	{
		path->addNode(n);
		n = n->father;
	}
	for(int i = 0, j = path->nodes.size()-1; i < j; ++i, --j)
		swap(path->nodes[i], path->nodes[j]);
	
	zip_path();
	path = short_path;
	// path->cost = cost;
}


int A_star::next_zip_node(int target)
{
	int l = target+1, r = path->nodes.size()-1;
	while(l < r)
	{
		int mid = ((l+r+1)>>1);
		if(can_zip(target, mid))
		{
			l = mid;
		}
		else
			r = mid-1;
	}
	return l;
}


void A_star::zip_path()
{
	short_path = make_shared<Path>();
	//起点加入
	short_path->nodes.emplace_back(path->nodes[0]);
	double cost = 0;
	int target = 0;
	while(target < path->nodes.size()-1)
	{
		int next_ = next_zip_node(target);
		short_path->nodes.emplace_back(path->nodes[next_]);
		cost += distance_bettwen_two(path->nodes[target]->x/2.0+0.25, path->nodes[target]->y/2.0+0.25, 
								path->nodes[next_]->x/2.0+0.25, path->nodes[next_]->y/2.0+0.25);
		target = next_;
	}
	short_path->cost = cost;
}

bool A_star::can_zip(int indx1, int indx2)
{
	if(abs(indx1 - indx2) == 1)
		return true;
	// line(int x0, int y0, int x1, int y1) {
	int x0 = path->nodes[indx1]->x, y0 = path->nodes[indx1]->y, x1 = path->nodes[indx2]->x, y1 = path->nodes[indx2]->y;
    int d_x = abs(x1 - x0);
    int d_y = abs(y1 - y0);
    int s_x = (x0 < x1) ? 1 : -1;
    int s_y = (y0 < y1) ? 1 : -1;
    int er = d_x - d_y;

    while (true) {
        // points.push_back({x0, y0});
		if(blocked(x0, y0))
			return false;
		for(auto &d : dir)
		{
			if(blocked(x0+d[0], y0+d[1]))
				return false;
		}

        if (x0 == x1 && y0 == y1) break;
        int er2 = 2 * er;
        if (er2 > -d_y) {
            er -= d_y;
            x0 += s_x;
        }
        if (er2 < d_x) {
            er += d_x;
            y0 += s_y;
        }
    }

    return true;
}

bool A_star::can_zip(int x0, int y0, int x1, int y1)
{
	// line(int x0, int y0, int x1, int y1) {
	// int x0 = path->nodes[indx1]->x, y0 = path->nodes[indx1]->y, x1 = path->nodes[indx2]->x, y1 = path->nodes[indx2]->y;
	if(crossed(x0, y0) || crossed(x1, y1))
		return false;
    int d_x = abs(x1 - x0);
    int d_y = abs(y1 - y0);
    int s_x = (x0 < x1) ? 1 : -1;
    int s_y = (y0 < y1) ? 1 : -1;
    int err = d_x - d_y;

    while (true) {
        // points.push_back({x0, y0});
		if(grid[x0][y0] == '#')
			return false;
		for(auto &d : dir)
		{
			if(!crossed(x0+d[0], y0+d[1]) && grid[x0+d[0]][y0+d[1]] == '#')
				return false;
		}
		
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -d_y) {
            err -= d_y;
            x0 += s_x;
        }
        if (e2 < d_x) {
            err += d_x;
            y0 += s_y;
        }
    }

    return true;
}