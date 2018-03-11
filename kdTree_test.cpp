#include "kdTree.h"
#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdlib>

using namespace crpp;
using namespace std;
using namespace cv;

void displayTree( Mat & M1, kdTreeNode<vector<double>> * rt ) {
    if(rt == NULL)
    {
        return;
    }
    Point p(rt->val[0]*100, rt->val[1]*100);
    circle(M1, p, 3, Scalar(255, 0, 0));
    if(rt->left != NULL)
    {
        Point np(rt->left->val[0]*100, rt->left->val[1]*100);
        line(M1, p, np, Scalar(255, 255, 0));
        displayTree(M1, rt->left);
    }
    if(rt->right != NULL)
    {
        Point np(rt->right->val[0]*100, rt->right->val[1]*100);
        line(M1, p, np, Scalar(255, 255, 0));
        displayTree(M1, rt->right);
    }
}

int main()
{
    kdTree<vector<double>, 2> tr;
    for(int i = 0; i<500; i++) {
        vector<double> exp1;
        double xx = double(rand()%60)/10;
        double yy = double(rand()%60)/10;
        exp1.push_back(xx);
        exp1.push_back(yy);
        tr.insert(exp1);
        Mat M(600, 600, CV_8UC3, Scalar(0,0,0));
        displayTree(M, tr.root);
        imshow( "Display window", M );
        waitKey(0);
    }
    return 0;
}
