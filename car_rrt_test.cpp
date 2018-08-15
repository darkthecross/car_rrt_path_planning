#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "car_rrt.h"

int main(int argc, char ** argv)
{
    if ( argc != 2 )
    {
        return -1;
    }
    crpp::CRRT solver;
    cv::Mat testMap = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if (testMap.rows == 0 || testMap.cols == 0)
    {
        return -1;
    }
    solver.initiateMapWithImage(testMap);
    solver.setStartEndStates(crpp::CarState(7, 7, 0), crpp::CarState(7, -7, PI));
    solver.planPath();
    return 0;
}
