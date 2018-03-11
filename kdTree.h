#ifndef KDTREE_H
#define KDTREE_H

#include <cmath>
//#include <cstddef>
#include <algorithm>
#include <limits>
#include <vector>

namespace crpp {

    template <class T>
    struct kdTreeNode {
        std::vector<double> val;
        kdTreeNode * left;
        kdTreeNode * right;
        kdTreeNode(T ele) {this->val = ele; this->left = NULL; this->right = NULL;};
    };


    template <class T, int N>
    class kdTree{
    private:
        kdTreeNode<T> * findNearestHelper (T newVal, kdTreeNode<T> * curNode, T lowerB, T upperB, double cur_min, int dps);
        void insertHelper(T newVal, kdTreeNode<T> * subTreeRoot, int dps);
        double lowerBoundInRange(T newVal, T lower, T upper);
        double NdDist(T N1, T N2);
        kdTreeNode<T> * selectNN(T newVal, kdTreeNode<T>* N1, kdTreeNode<T> * N2);
    public:
        kdTreeNode<T> * root;
        kdTree() {root = NULL;}
        void insert(T newVal);
        kdTreeNode<T> * findNearest (T newVal);
    };
}

#endif
