#ifndef CAR_RRT_H
#define CAR_RRT_H

#include <cmath>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

namespace crpp {

typedef struct car_state {
  double x;
  double y;
  double theta;
  car_state(double xx=0, double yy=0, double tt=0) {
    x = xx;
    y = yy;
    theta = tt;
  }
} CarState;

typedef struct car_state_node {
    CarState state;
    std::vector<car_state_node *> children;
    double distFromRoot;
    car_state_node() {state = CarState(0,0,0); children = std::vector<car_state_node *>(); distFromRoot = 0;}
    car_state_node(CarState cs) {state = cs; children = std::vector<car_state_node *>(); distFromRoot = 0;}
} CarStateNode;

class CRRT {
private:
  double crrt_resolution;
  double crrt_rmin;
  CarState crrt_start_state;
  CarState crrt_end_state;
  // map size: 1001 * 1001
  // default: empty
  std::vector<std::vector<bool>> environment_map;

public:
  CRRT();
  CRRT(CarStateNode* root);
  std::vector<CarState> localPlanner(const CarState start, const CarState end,
                                const double rmin, const double resolution);
  void initiateMapWithImage(cv::Mat mapImage);
  void setStartEndStates(const CarState start_state, const CarState end_state);
  bool expandTree(CarState target);
  bool planPath();
  bool localPathAvailable(std::vector<CarState> localPath);
};
} // namespace crpp

#endif /* end of include guard:  */
