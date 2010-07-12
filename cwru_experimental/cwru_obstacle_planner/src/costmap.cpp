#include<harlie_obstacle_planner/costmap.h>



CostMap::CostMap(int x,int y)
{
    printf("Making a map %d = x %d = y",x,y);
    this->map = (CostNode *)malloc(sizeof(void *)*x*y);
    for(int ix = 0; ix < x;ix++)
    {
        for(int iy = 0; iy < y;iy++)
        {
            //this->map[ix*x+y] = new CostNode(-1);
        }
    }
}
