#include "kdTree.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <time.h>
#include <string>
#include <iostream>
#include <cmath>

using namespace crpp;
using namespace std;
using namespace cv;

typedef struct car_state
{
    double x;
    double y;
    double theta;
}CarState;

vector<car_state> localPlanner(const CarState start,
    const CarState end, const double rmin, const double resolution)
{
    // O1: (start.x - r \sin{start.theta}, start.y - r \cos{start.theta} ), r1
    vector<car_state> traj;

    //check if solution exists
    double start_min_circle_x = start.x - rmin * math.sin(start.theta);
    double start_min_circle_y = start.y + rmin * math.cos(start.theta);
    double vec_1_x = start_min_circle_x - end.x;
    double vec_1_y = start_min_circle_y - end.y;
    double vec_2_x = -math.sin(end.theta);
    double vec_2_y = math.cos(end.theta);
    double dist_start_min_circle_origin_to_end_line = \
    vec_1_x * vec_2_x + vec_1_y * vec_2_y;
    // dist_start_min_circle_origin_to_end_line < r_min, no solution.
    if (dist_start_min_circle_origin_to_end_line < rmin)
    {
        return traj;
    }
    else
    {
        // have solution. 
    }
}
