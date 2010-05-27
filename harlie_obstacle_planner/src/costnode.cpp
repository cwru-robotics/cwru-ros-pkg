#include<harlie_obstacle_planner/costmap.h>

CostNode::CostNode()
{

}

CostNode::CostNode(float x)
{
    this->cost = x;
}

float CostNode::getCost()
{
    return this->cost;
}

