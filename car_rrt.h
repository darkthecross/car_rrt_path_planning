#ifndef CAR_RRT_H
#define CAR_RRT_H

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>
#include <limits>
#include <cstdlib>

#define PI 3.1415926535897932

namespace crpp {

double round_to_pi(const double raw_theta) {
  double tmp_theta = raw_theta;
  while (tmp_theta < -PI) {
    tmp_theta += 2 * PI;
  }
  while (tmp_theta > PI) {
    tmp_theta -= 2 * PI;
  }
  return tmp_theta;
}

double random_ranged(double lower, double upper)
{
    double f = (double)rand() / RAND_MAX;
    return lower + f * (upper - lower);
}

typedef struct car_state {
  double x;
  double y;
  double theta;
  car_state(double xx = 0, double yy = 0, double tt = 0) {
    x = xx;
    y = yy;
    theta = tt;
  }
} CarState;

typedef struct car_state_node {
  CarState state;
  car_state_node * parent;
  std::vector<car_state_node *> children;
  double distFromRoot;
  car_state_node() {
      parent = nullptr;
    state = CarState(0, 0, 0);
    children = std::vector<car_state_node *>();
    distFromRoot = 0;
  }
  car_state_node(CarState cs) {
      parent = nullptr;
    state = cs;
    children = std::vector<car_state_node *>();
    distFromRoot = 0;
  }
} CarStateNode;

class CRRT {
private:
  double crrt_resolution;
  double crrt_rmin;
  CarState crrt_start_state;
  CarState crrt_end_state;
  double map_x_min;
  double map_y_min;
  double map_x_max;
  double map_y_max;
  // map size: 1000 * 1000
  // map coordinate: [-10, 10]
  // default: empty
  std::vector<std::vector<bool>> environment_map;
  CarStateNode * root;
  std::vector<CarStateNode *> allTreeNodes;

public:
  CRRT();
  std::vector<CarState> localPlanner(const CarState start, const CarState end,
                                     const double rmin,
                                     const double resolution);
  void initiateMapWithImage(const cv::Mat mapImage);
  void setStartEndStates(const CarState start_state, const CarState end_state);
  bool expandTree(const CarState target);
  std::vector<CarState> planPath();
  bool localPathAvailable(const std::vector<CarState> localPath);
};
} // namespace crpp

#endif /* end of include guard:  */
