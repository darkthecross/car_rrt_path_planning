#include "car_rrt.h"
#include <algorithm>

cv::Point crpp::CRRT::stateToCVPoint(CarState tmpState) {
  return cv::Point((map_y_max - tmpState.y) / (map_y_max - map_y_min) * 1000,
                   (tmpState.x - map_x_min) / (map_x_max - map_x_min) * 1000);
}

crpp::CRRT::CRRT() {
  srand(7826);
  crrt_resolution = 0.1;
  crrt_rmin = 0.5;
  map_x_min = -10;
  map_y_min = -10;
  map_x_max = 10;
  map_y_max = 10;
  root = nullptr;
  allTreeNodes = std::vector<CarStateNode *>();
  environment_map =
      std::vector<std::vector<bool>>(1000, std::vector<bool>(1000, false));
}

double crpp::CRRT::round_to_mp_pi(const double raw_theta) {
  double tmp_theta = raw_theta;
  while (tmp_theta < -PI) {
    tmp_theta += 2 * PI;
  }
  while (tmp_theta > PI) {
    tmp_theta -= 2 * PI;
  }
  return tmp_theta;
}

double crpp::CRRT::random_range(double lower, double upper) {
  double f = (double)rand() / RAND_MAX;
  return lower + f * (upper - lower);
}

void crpp::CRRT::initiateMapWithImage(const cv::Mat mapImage) {
  cv::Mat im_gray; // no nead to load the Mat with anything when declaring it.
  debugMap = mapImage;
  cv::cvtColor(mapImage, im_gray, CV_RGB2GRAY);
  // INSTEAD of the above two lines you could have cv::Mat im_gray =
  // imread("img1.png",CV_LOAD_IMAGE_GRAYSCALE);

  // the following is an alternative to Mat img_bw = im_gray > 128
  cv::Mat img_bw;
  cv::threshold(im_gray, img_bw, 128.0, 255.0, cv::THRESH_BINARY);

  cv::Mat im_resized;
  cv::resize(img_bw, im_resized, cv::Size(1000, 1000));
  for (size_t i = 0; i < 1000; i++) {
    for (size_t j = 0; j < 1000; j++) {
      cv::Scalar tmpColor = im_resized.at<uchar>(j, i);
      environment_map[i][j] = (tmpColor[0] == 255);
    }
  }
}

void crpp::CRRT::setStartEndStates(const CarState start_state,
                                   const CarState end_state) {
  this->crrt_start_state = start_state;
  this->crrt_end_state = end_state;
  this->root = new CarStateNode(this->crrt_start_state);
  allTreeNodes.push_back(this->root);
}

crpp::CarStateNode *crpp::CRRT::expandTree(const CarState target) {
  double cur_min_dist = std::numeric_limits<double>::max();
  CarStateNode *cur_min_ptr = nullptr;
  std::vector<CarStateNode *>::iterator target_idx = allTreeNodes.end();
  for (std::vector<CarStateNode *>::iterator csnptr = allTreeNodes.begin();
       csnptr != allTreeNodes.end(); csnptr++) {
    if ((*csnptr)->distFromRoot > cur_min_dist) {
      target_idx = csnptr;
      break;
    }
    std::vector<crpp::CarState> tmp_path = localPlanner(
        (*csnptr)->state, target, this->crrt_rmin, this->crrt_resolution);
    if (!localPathAvailable(tmp_path)) {
      continue;
    }
    if (tmp_path.size() > 0) {
      double tmp_dist =
          (*csnptr)->distFromRoot + tmp_path.size() * this->crrt_resolution;
      if (tmp_dist < cur_min_dist) {
        cur_min_ptr = (*csnptr);
        cur_min_dist = tmp_dist;
      }
    }
  }
  if (cur_min_ptr == nullptr) {
    return nullptr;
  } else {
    CarStateNode *newNode = new CarStateNode(target);
    newNode->distFromRoot = cur_min_dist;
    newNode->parent = cur_min_ptr;
    allTreeNodes.insert(target_idx - 1, newNode);
    cur_min_ptr->children.push_back(newNode);

    std::vector<crpp::CarState> tmp_path = localPlanner(
        cur_min_ptr->state, target, this->crrt_rmin, this->crrt_resolution);

    for (int i = 0; i < tmp_path.size() - 1; i++) {
      cv::Point s1 = stateToCVPoint(tmp_path[i]);
      cv::Point e1 = stateToCVPoint(tmp_path[i + 1]);
      cv::line(debugMap, s1, e1, cv::Scalar(0, 255, 0), 3);
    }

    return newNode;
  }
}

std::vector<crpp::CarState> crpp::CRRT::planPath() {
  namedWindow("Display window", cv::WINDOW_AUTOSIZE);

  cv::Point s1 = stateToCVPoint(crrt_start_state);
  cv::Point e1 = stateToCVPoint(crrt_end_state);

  cv::circle(debugMap, s1, 5, cv::Scalar(255, 0, 0), CV_FILLED);
  cv::circle(debugMap, e1, 5, cv::Scalar(0, 0, 255), CV_FILLED);
  cv::imshow("Display window", debugMap);
  cv::waitKey(0);

  int i = 0;

  CarStateNode *finalNode = nullptr;

  for (i = 0; i < 10000; i++) {
    //std::cout << i << std::endl;
    CarState tmpTarget;
    if (i > 0 && i % 10 == 0) {
      tmpTarget = crrt_end_state;
    } else {
      tmpTarget.x = random_range(map_x_min, map_x_max);
      tmpTarget.y = random_range(map_y_min, map_y_max);
      tmpTarget.theta = random_range(-PI, PI);
    }

    CarStateNode *extendResult = expandTree(tmpTarget);
    while (extendResult == nullptr) {
      tmpTarget.x = random_range(map_x_min, map_x_max);
      tmpTarget.y = random_range(map_y_min, map_y_max);
      tmpTarget.theta = random_range(-PI, PI);
      extendResult = expandTree(tmpTarget);

      cv::Point s2 = stateToCVPoint(tmpTarget);

      cv::circle(debugMap, s2, 5, cv::Scalar(0, 255, 0), CV_FILLED);

      cv::imshow("Display window", debugMap);
      cv::waitKey(0);
    }
    cv::imshow("Display window", debugMap);
    cv::waitKey(0);
    if (tmpTarget == crrt_end_state) {
      finalNode = extendResult;
      break;
    }
  }
  if (i == 10000) {
    // failed
    return std::vector<crpp::CarState>();
  } else {
    if (finalNode == nullptr) {
      std::cout << "nptr" << std::endl;
      return std::vector<crpp::CarState>();
    } else {
      std::vector<crpp::CarState> finalPath;
      while (finalNode->parent != nullptr) {
        finalPath.insert(finalPath.begin(), finalNode->state);
        finalNode = finalNode->parent;
      }
      finalPath.insert(finalPath.begin(), finalNode->state);

      for (int j = 0; j < finalPath.size() - 1; j++) {
        std::vector<CarState> subPath =
            localPlanner(finalPath[j], finalPath[j + 1], this->crrt_rmin,
                         this->crrt_resolution);
        for (int k = 0; k < subPath.size() - 1; k++) {
          cv::Point s1 = stateToCVPoint(subPath[k]);
          cv::Point e1 = stateToCVPoint(subPath[k + 1]);
          cv::line(debugMap, s1, e1, cv::Scalar(0, 0, 0), 3);
        }
      }
      cv::imshow("Display window", debugMap);
      cv::imwrite("img1_solved.png", debugMap);
      cv::waitKey(0);
      return finalPath;
    }
  }
}

bool crpp::CRRT::localPathAvailable(const std::vector<CarState> localPath) {
  double available = true;
  for (auto s : localPath) {
    int sx = int(floor((map_y_max - s.y) / (map_y_max - map_y_min) * 1000));
    int sy = int(floor((s.x - map_x_min) / (map_x_max - map_x_min) * 1000));
    if (sx < 0 || sx >= 1000 || sy < 0 || sy >= 1000) {
      available = false;
      break;
    } else if (!environment_map[sx][sy]) {
      available = false;
      break;
    } else {
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
      double start_theta = round_to_mp_pi(start.theta - PI / 2);
      double end_theta = round_to_mp_pi(atan2(
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
      double end_theta_2 = round_to_mp_pi(end.theta - PI / 2);
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
      double start_theta = round_to_mp_pi(start.theta - PI / 2);

      double start_circle_x = start.x - r_2 * sin(start.theta);
      double start_circle_y = start.y + r_2 * cos(start.theta);

      double end_theta =
          round_to_mp_pi(atan2(end_min_circle_y - start_circle_y,
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
      double end_theta_2 = round_to_mp_pi(end.theta - PI / 2);
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
