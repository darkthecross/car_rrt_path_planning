#include "kdTree.h"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <time.h>
#include <vector>

using namespace crpp;
using namespace std;
using namespace cv;

#define PI 3.1415926535897932

typedef struct car_state {
  double x;
  double y;
  double theta;
  car_state(double xx, double yy, double tt) {
    x = xx;
    y = yy;
    theta = tt;
  }
} CarState;

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

Point coordinate_to_img(double x, double y) {
  return Point(50 * x + 400, 400 - 50 * y);
}

vector<CarState> localPlanner(const CarState start, const CarState end,
                              const double rmin, const double resolution,
                              Mat &M) {
  // O1: (start.x - r \sin{start.theta}, start.y - r \cos{start.theta} ), r1
  vector<CarState> solution1, solution2;
  // solution 1
  // check if solution exists
  {
    double start_min_circle_x = start.x - rmin * sin(start.theta);
    double start_min_circle_y = start.y + rmin * cos(start.theta);
    // have solution.
    double frac_bot = -2 * (end.x - start_min_circle_x) * sin(end.theta) +
                      2 * (end.y - start_min_circle_y) * cos(end.theta) +
                      2 * rmin;
    double frac_top = pow(rmin, 2) - pow(end.x - start_min_circle_x, 2) -
                      pow(end.y - start_min_circle_y, 2);
    double r_1 = frac_top / frac_bot;
    if (r_1 > rmin) {
      solution1.push_back(start);
      // part of circle 1
      double delta_theta_1 = resolution / rmin;
      double start_theta = round_to_pi(start.theta - PI / 2);
      double end_theta = round_to_pi(atan2(
          start.y + rmin * cos(start.theta) - end.y - r_1 * cos(end.theta),
          start.x - rmin * sin(start.theta) - end.x + r_1 * sin(end.theta)));
      double direct = sin(end_theta - start_theta);
      double direct_c = cos(end_theta - start_theta);
      end_theta = start_theta + atan2(direct, direct_c);
      double cur_theta = start_theta;
      if (direct < 0) {
        while (cur_theta > end_theta) {
          cur_theta -= delta_theta_1;
          solution1.push_back(CarState(
              start_min_circle_x + rmin * cos(cur_theta),
              start_min_circle_y + rmin * sin(cur_theta), PI / 2 + cur_theta));
        }
      } else {
        while (cur_theta < end_theta) {
          cur_theta += delta_theta_1;
          solution1.push_back(CarState(
              start_min_circle_x + rmin * cos(cur_theta),
              start_min_circle_y + rmin * sin(cur_theta), PI / 2 + cur_theta));
        }
      }
      // part of circle 2
      double delta_theta_2 = resolution / r_1;
      double start_theta_2 = end_theta;
      double end_theta_2 = round_to_pi(end.theta - PI / 2);
      cout << "end_theta_2 " << end_theta_2 << endl;
      double direct_2 = sin(end_theta_2 - start_theta_2);
      double direct_c_2 = cos(end_theta_2 - start_theta_2);
      end_theta_2 = start_theta_2 + atan2(direct_2, direct_c_2);
      double cur_theta_2 = start_theta_2;
      cout << "end_theta_2 " << end_theta_2
           << " start_theta_2: " << start_theta_2 << endl;
      double end_circle_x = end.x - r_1 * sin(end.theta);
      double end_circle_y = end.y + r_1 * cos(end.theta);

      if (direct_2 < 0) {
        while (cur_theta_2 > end_theta_2) {
          cur_theta_2 -= delta_theta_2;
          solution1.push_back(CarState(end_circle_x + r_1 * cos(cur_theta_2),
                                       end_circle_y + r_1 * sin(cur_theta_2),
                                       PI / 2 + cur_theta_2));
        }
      } else {
        while (cur_theta_2 < end_theta_2) {
          cur_theta_2 += delta_theta_2;
          solution1.push_back(CarState(end_circle_x + r_1 * cos(cur_theta_2),
                                       end_circle_y + r_1 * sin(cur_theta_2),
                                       PI / 2 + cur_theta_2));
        }
      }
    }
  }
  {
    double end_min_circle_x = end.x - rmin * sin(end.theta);
    double end_min_circle_y = end.y + rmin * cos(end.theta);
    // have solution.
    double frac_bot_2 = -2 * (start.x - end_min_circle_x) * sin(start.theta) +
                        2 * (start.y - end_min_circle_y) * cos(start.theta) +
                        2 * rmin;
    double frac_top_2 = pow(rmin, 2) - pow(start.x - end_min_circle_x, 2) -
                        pow(start.y - end_min_circle_y, 2);
    double r_2 = frac_top_2 / frac_bot_2;

    if (r_2 > rmin) {
      solution2.push_back(start);
      // part of circle 1
      double delta_theta_1 = resolution / r_2;
      double start_theta = round_to_pi(start.theta - PI / 2);

      double start_circle_x = start.x - r_2 * sin(start.theta);
      double start_circle_y = start.y + r_2 * cos(start.theta);
      cout << "r_2" << r_2 << endl;
      cout << "start_circle_x " << start_circle_x << "start_circle_y"
           << start_circle_y << endl;

      double end_theta = round_to_pi(atan2(end_min_circle_y - start_circle_y,
                                           end_min_circle_x - start_circle_x));

      double direct = sin(end_theta - start_theta);
      double direct_c = cos(end_theta - start_theta);
      end_theta = start_theta + atan2(direct, direct_c);
      double cur_theta = start_theta;
      if (direct < 0) {
        while (cur_theta > end_theta) {
          cur_theta -= delta_theta_1;
          solution2.push_back(CarState(start_circle_x + r_2 * cos(cur_theta),
                                       start_circle_y + r_2 * sin(cur_theta),
                                       PI / 2 + cur_theta));
        }
      } else {
        while (cur_theta < end_theta) {
          cur_theta += delta_theta_1;
          solution2.push_back(CarState(start_circle_x + r_2 * cos(cur_theta),
                                       start_circle_y + r_2 * sin(cur_theta),
                                       PI / 2 + cur_theta));
        }
      }
      // part of circle 2
      double delta_theta_2 = resolution / rmin;
      double start_theta_2 = end_theta;
      double end_theta_2 = round_to_pi(end.theta - PI / 2);
      cout << "start_theta_2 " << start_theta_2 << " end_theta_2 "
           << end_theta_2 << endl;
      double direct_2 = sin(end_theta_2 - start_theta_2);
      double direct_c_2 = cos(end_theta_2 - start_theta_2);
      end_theta_2 = start_theta_2 + atan2(direct_2, direct_c_2);
      double cur_theta_2 = start_theta_2;

      if (direct_2 < 0) {
        while (cur_theta_2 > end_theta_2) {
          cur_theta_2 -= delta_theta_2;
          solution2.push_back(
              CarState(end_min_circle_x + rmin * cos(cur_theta_2),
                       end_min_circle_y + rmin * sin(cur_theta_2),
                       PI / 2 + cur_theta_2));
        }
      } else {
        while (cur_theta_2 < end_theta_2) {
          cur_theta_2 += delta_theta_2;
          solution2.push_back(
              CarState(end_min_circle_x + rmin * cos(cur_theta_2),
                       end_min_circle_y + rmin * sin(cur_theta_2),
                       PI / 2 + cur_theta_2));
        }
      }
    }
  }
  if( solution1.size() == 0 )
  {
      return solution2;
  }
  else if(solution2.size() == 0)
  {
      return solution1;
  }
  else
  {
      if(solution2.size() > solution1.size())
      {
          return solution1;
      }
      else
      {
          return solution2;
      }
  }
}

int main() {

  double rmin = 1;
  double resolution = 0.05;

  namedWindow("Display window",
              WINDOW_AUTOSIZE); // Create a window for display.

  srand(time(NULL));

  /*
    CarState start =
        CarState(double(rand() % 120) / 20 - 8.0, double(rand() % 120) / 20
    - 4.0, double((rand() % 180)) / PI); CarState end = CarState(double(rand() %
    120) / 20 - 8.0, double(rand() % 120) / 20 - 4.0, double((rand() % 180)) /
    PI);

  */
  CarState start(1, 0, PI / 2);
  CarState end =
      CarState(double(rand() % 120) / 40 - 7.0, double(rand() % 120) / 40+2.0,
               double((rand() % 180)) / PI);

  Mat M1(600, 600, CV_8UC3, Scalar(0, 0, 0));
  vector<CarState> sols = localPlanner(start, end, rmin, resolution, M1);

  if (sols.size() != 0) {
    for (auto j = 0; j < sols.size() - 1; j++) {
      line(M1, coordinate_to_img(sols[j].x, sols[j].y),
           coordinate_to_img(sols[j + 1].x, sols[j + 1].y),
           Scalar(255, 255, 255));
    }
  }

  imshow("Display window", M1);
  imwrite("img1.png", M1);
  waitKey(0);
  return 0;
}
