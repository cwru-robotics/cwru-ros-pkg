#include<harlie_obstacle_planner/costmap.h>

CostNode::CostNode()
{

}

bool CostNode::operator < (const CostNode& lhs, const CostNode& rhs) const
{
     return (lhs.cost<rhs.cost);
}

bool CostNode::operator() (const CostNode& lhs, const CostNode& rhs) const
{
    if(lhs.x == rhs.x)
    {
        if(lhs.y == rhs.y)
        {
            return true;
        }
    }
    return false;
}
