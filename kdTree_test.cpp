#include "kdTree.h"
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <string>
#include <time.h>
#include <vector>

using namespace crpp;
using namespace std;
using namespace cv;

void displayTree(Mat &M1, kdTreeNode<vector<double>> *rt, vector<double> lb,
                 vector<double> ub, int dps) {
  if (rt == NULL) {
    return;
  }
  Point p(rt->val[0] * 100, rt->val[1] * 100);
  circle(M1, p, 3, Scalar(255, 0, 0));
  Point p1(lb[0] * 100 + 1, lb[1] * 100 + 1);
  Point p2(lb[0] * 100 + 1, ub[1] * 100 + 1);
  Point p3(ub[0] * 100 + 1, lb[1] * 100 + 1);
  Point p4(ub[0] * 100 + 1, ub[1] * 100 + 1);
  line(M1, p1, p2, Scalar(0, 0, 255));
  line(M1, p1, p3, Scalar(0, 0, 255));
  line(M1, p2, p4, Scalar(0, 0, 255));
  line(M1, p3, p4, Scalar(0, 0, 255));
  if (rt->left != NULL) {
    Point np(rt->left->val[0] * 100 + 1, rt->left->val[1] * 100 + 1);
    // line(M1, p, np, Scalar(255, 255, 0));
    vector<double> ub1 = ub;
    ub1[dps % 2] = rt->val[dps % 2];
    displayTree(M1, rt->left, lb, ub1, dps + 1);
  }
  if (rt->right != NULL) {
    Point np(rt->right->val[0] * 100 + 1, rt->right->val[1] * 100 + 1);
    // line(M1, p, np, Scalar(255, 255, 0));
    vector<double> lb1 = lb;
    lb1[dps % 2] = rt->val[dps % 2];
    displayTree(M1, rt->right, lb1, ub, dps + 1);
  }
}

int main() {
  srand(time(NULL));
  kdTree<vector<double>, 2> tr;
  for (int i = 0; i < 500; i++) {
    vector<double> exp1;
    double xx = double(rand() % 60) / 10;
    double yy = double(rand() % 60) / 10;
    exp1.push_back(xx);
    exp1.push_back(yy);
    kdTreeNode<vector<double>> *nnNode = tr.findNearest(exp1);
    tr.insert(exp1);
    Mat M(602, 602, CV_8UC3, Scalar(0, 0, 0));
    vector<double> lb, ub;
    for (int i = 0; i < 2; i++) {
      lb.push_back(0);
      ub.push_back(6);
    }
    displayTree(M, tr.root, lb, ub, 0);
    if (nnNode != NULL) {
      Point nn(nnNode->val[0] * 100 + 1, nnNode->val[1] * 100 + 1);
      circle(M, nn, 3, Scalar(255, 255, 255));
      Point nn1(xx * 100 + 1, yy * 100 + 1);
      circle(M, nn1, 3, Scalar(0, 255, 255));
    }
    imshow("Display window", M);
    imwrite(string("imgs/") + to_string(i) + string(".jpg"), M);
    waitKey(0);
  }
  return 0;
}
