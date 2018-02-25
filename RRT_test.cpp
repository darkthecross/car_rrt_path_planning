#include "RRT.h"

using namespace std;

using namespace crpp;

int main()
{
    vector<double> tmpvec;
    tmpvec.push_back(1);
    tmpvec.push_back(2);
    RRT crrt(tmpvec);
    crrt.speak();
    vector<double> exp1;
    exp1.push_back(2);
    exp1.push_back(2);
    crrt.expandTree(exp1);
    crrt.speak();
    vector<double> exp2;
    exp2.push_back(1.5);
    exp2.push_back(1);
    crrt.expandTree(exp2);
    crrt.speak();
    return 0;
}
