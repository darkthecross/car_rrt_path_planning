#include "car_rrt.h"

crpp::CRRT::CRRT() {
    srand(0);
  crrt_resolution = 0.1;
  crrt_rmin = 0.5;
  map_x_min = -10;
  map_y_min = -10;
  map_x_max = 10;
  map_y_max = 10;
  root = nullptr;
  allTreeNodes = std::vector<CarStateNode *>();
  environment_map = std::vector<std::vector<bool>>(1000, std::vector<bool>(1000, false));
}

void crpp::CRRT::initiateMapWithImage(const cv::Mat mapImage) {
    cv::Mat im_gray;   // no nead to load the Mat with anything when declaring it.
    cv::cvtColor(mapImage, im_gray,CV_RGB2GRAY);
    // INSTEAD of the above two lines you could have cv::Mat im_gray = imread("img1.png",CV_LOAD_IMAGE_GRAYSCALE);

    // the following is an alternative to Mat img_bw = im_gray > 128
    cv::Mat img_bw;
    cv::threshold(im_gray, img_bw, 128.0, 255.0, cv::THRESH_BINARY);
    

}

void crpp::CRRT::setStartEndStates(const CarState start_state,
                                   const CarState end_state) {
  this->crrt_start_state = start_state;
  this->crrt_end_state = end_state;
  this->root = new CarStateNode(this->crrt_start_state);
  allTreeNodes.push_back(this->root);
}

bool crpp::CRRT::expandTree(const CarState target) {
    double cur_min_dist = std::numeric_limits<double>::max();
    CarStateNode * cur_min_ptr = nullptr;
    for (auto csnptr : allTreeNodes)
    {
        if (csnptr->distFromRoot > cur_min_dist)
        {
            continue;
        }
        std::vector<crpp::CarState> tmp_path = localPlanner(csnptr->state, target, this->crrt_rmin, this->crrt_resolution);
        if (!localPathAvailable(tmp_path))
        {
            continue;
        }
        if(tmp_path.size() > 0)
        {
            double tmp_dist = csnptr->distFromRoot + tmp_path.size() * this->crrt_resolution;
            if (tmp_dist < cur_min_dist)
            {
                cur_min_ptr = csnptr;
                cur_min_dist = tmp_dist;
            }
        }
    }
    if (cur_min_ptr == nullptr)
    {
        return false;
    } else {
        CarStateNode * newNode = new CarStateNode(target);
        newNode->distFromRoot = cur_min_dist;
        newNode->parent = cur_min_ptr;
        cur_min_ptr->children.push_back(newNode);
        return true;
    }
}

std::vector<crpp::CarState> crpp::CRRT::planPath() {
  return std::vector<crpp::CarState>();
}

bool crpp::CRRT::localPathAvailable(const std::vector<CarState> localPath) {
    double available = true;
    for (auto s : localPath)
    {
        int sx = int( floor((s.x-map_x_min) / (map_x_max - map_x_min) * 1000 ));
        int sy = int( floor((map_y_max-s.y) / (map_y_max - map_y_min) * 1000 ));
        if(sx < 0 || sx >= 1000 || sy < 0 || sy >= 1000)
        {
            available = false;
            break;
        }
        else if( !environment_map[sx][sy] )
        {
            available = false;
            break;
        }
        else
        {
            continue;
        }
    }
  return available;
}

std::vector<crpp::CarState> crpp::CRRT::localPlanner(const CarState start,
                                                     const CarState end,
                                                     const double rmin,
                                                     const double resolution) {
  std::vector<CarState> solution1, solution2;
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
      double direct_2 = sin(end_theta_2 - start_theta_2);
      double direct_c_2 = cos(end_theta_2 - start_theta_2);
      end_theta_2 = start_theta_2 + atan2(direct_2, direct_c_2);
      double cur_theta_2 = start_theta_2;
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
  if (solution1.size() == 0) {
    return solution2;
  } else if (solution2.size() == 0) {
    return solution1;
  } else {
    if (solution2.size() > solution1.size()) {
      return solution1;
    } else {
      return solution2;
    }
  }
}
