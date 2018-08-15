#ifndef CAR_RRT_H
#define CAR_RRT_H

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <vector>

#define PI 3.1415926535897932

namespace crpp {

typedef struct car_state {
  double x;
  double y;
  double theta;
  car_state(double xx = 0, double yy = 0, double tt = 0) {
    x = xx;
    y = yy;
    theta = tt;
  }
  bool operator==(const car_state other) {
    return other.x == this->x && other.y == this->y &&
           other.theta == this->theta;
  }
} CarState;

typedef struct car_state_node {
  CarState state;
  car_state_node *parent;
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

class mycomparison {
  bool reverse;

public:
  mycomparison(const bool &revparam = false) { reverse = revparam; }
  bool operator()(const CarStateNode *lhs, const CarStateNode *rhs) const {
    if (reverse)
      return (lhs->distFromRoot > rhs->distFromRoot);
    else
      return (lhs->distFromRoot > rhs->distFromRoot);
  }
};

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
  // avaliable: 1; block: 0
  std::vector<std::vector<bool>> environment_map;
  CarStateNode *root;
  std::vector<CarStateNode *> allTreeNodes;
  cv::Mat debugMap;

public:
  CRRT();
  cv::Point stateToCVPoint(CarState tmpState);
  double round_to_mp_pi(const double raw_theta);
  double random_range(double lower, double upper);
  std::vector<CarState> localPlanner(const CarState start, const CarState end,
                                     const double rmin,
                                     const double resolution);
  void initiateMapWithImage(const cv::Mat mapImage);
  void setStartEndStates(const CarState start_state, const CarState end_state);
  CarStateNode * expandTree(const CarState target);
  std::vector<CarState> planPath();
  bool localPathAvailable(const std::vector<CarState> localPath);
};
} // namespace crpp

#endif /* end of include guard:  */
