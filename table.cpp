#include "table.hh"
#include "arg.hh"
#include "toll.hh"
#include <iostream>
vector<Table> tables;
vector<Table> d_tables;

Table::Table (int id, int table_type, int product_type, int sum_component, double x, double y) :
			id(id), table_type(table_type), remain_time(-1),  have_product(0), product_type(product_type), remain_component(sum_component),
			sum_component(sum_component), x(x), y(y),
			want_buy(product_type_num+1, false), 
			hava_this_component(product_type_num+1, false),
			shut_down(INT32_MAX/2), occupation(0) {
				for(auto a : the_table_want_buy[table_type])
				{
					want_buy[a] = true;
				}
			}

void Table::set_state(double remain_time, int component_state, int have_product)
{
	this->remain_time = remain_time;
	this->have_product = have_product;
	
	//更新配件信息
	remain_component = sum_component;
	for(auto a : the_table_want_buy[table_type])
	{
		if((1<<a) & component_state)
		{
			--remain_component;
			this->hava_this_component[a] = true;
		}
		else
		{
			this->hava_this_component[a] = false;
		}
	}
}