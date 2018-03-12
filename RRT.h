#ifndef RRT_H
#define RRT_H

#include <vector>
#include <iostream>
#include <cmath>
#include "kdTree.h"

namespace crpp{

    template<class T, int N>
    class RRT: public kdTree<T, N>{
    private:
        double stepSize;
    public:
        RRT();
        RRT(T root_coord);
        double nodeDistance(const kdTreeNode<T>* node1, const kdTreeNode<T>* node2);
        bool expandTree(T target);
        bool expandFromNode(kdTreeNode<T> * fromNode, kdTreeNode<T> * tarNode);
        bool collision(T target);
    };
}

#endif /* end of include guard:  */
