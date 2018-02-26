#include "RRT.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>

using namespace cv;

using namespace std;

using namespace crpp;

void displayTree( Mat & M1, TreeNode * rt ) {
    Point p(rt->coordinate[0]*100, rt->coordinate[1]*100);
    circle(M1, p, 3, Scalar(255, 0, 0));
    for(int i = 0; i<rt->children.size(); i++) {
        Point np(rt->children[i]->coordinate[0]*100, rt->children[i]->coordinate[1]*100);
        line(M1, p, np, Scalar(255, 255, 0));
        displayTree(M1, rt->children[i]);
    }
}

int main()
{
    vector<double> tmpvec;
    tmpvec.push_back(1);
    tmpvec.push_back(2);
    RRT crrt(tmpvec);

    namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.

    for(int i = 0; i<500; i++) {
        vector<double> exp1;
        double xx = double(rand()%60)/10;
        double yy = double(rand()%60)/10;
        exp1.push_back(xx);
        exp1.push_back(yy);
        crrt.expandTree(exp1);
        Mat M(600, 600, CV_8UC3, Scalar(0,0,0));
        displayTree(M, crrt.root);

        imshow( "Display window", M );
        waitKey(0);
    }

    return 0;
}
