#include "RRT.h"

using namespace crpp;

template<class T, int N>
RRT<T, N>::RRT() {
    this->stepSize = 0.1;
}

template<class T, int N>
RRT<T, N>::RRT(T root_coord) {
    this->stepSize = 0.1;
    this->root = new kdTreeNode<T>(root_coord);
}

template<class T, int N>
bool RRT<T, N>::expandTree(T target) {
    // find nearest node
    kdTreeNode<T> * nearest =  kdTree<T,N>::findNearest(target);
    kdTreeNode<T> * tarNode = new kdTreeNode<T>(target);
    return expandFromNode(nearest, tarNode);
}

template<class T, int N>
bool RRT<T, N>::expandFromNode(kdTreeNode<T> * fromNode, kdTreeNode<T> * tarNode) {
    double totalDist = nodeDistance(fromNode, tarNode);
    if(totalDist < this->stepSize) {
        kdTreeNode<T> * tmpNode = kdTree<T,N>::insert(tarNode->val);
        tmpNode->parent = fromNode;
        fromNode->children.push_back(tmpNode);
        return true;
    } else {
        T tmpNodeVal;
        for(int i = 0; i<N; i++)
        {
            tmpNodeVal.push_back( fromNode->val[i] + (tarNode->val[i] - fromNode->val[i]) * this->stepSize / totalDist );
        }
        if(this->collision(tmpNodeVal)) {
            return false;
        } else {
            kdTreeNode<T> * tmpNode = kdTree<T,N>::insert(tmpNodeVal);
            tmpNode->parent = fromNode;
            fromNode->children.push_back(tmpNode);
            return expandFromNode(tmpNode, tarNode);
        }
    }
}

template<class T, int N>
bool RRT<T, N>::collision(T target) {
    return false;
}

template<class T, int N>
double RRT<T, N>::nodeDistance(const kdTreeNode<T>* node1, const kdTreeNode<T>* node2) {
    double dst = 0;
    for(int i = 0; i<N; i++)
    {
        dst += (node1->val[i] - node2->val[i]) * (node1->val[i] - node2->val[i]);
    }
    return std::sqrt(dst);
}

template class RRT<std::vector<double>, 2>;

/*
void RRT::speak() {
    print_tree(this->root);
}

void RRT::print_tree(kdTreeNode<T> * r) {
    for(int i = 0; i<this->dim; i++) {
        std::cout<<r->val[i]<<" ";
    }
    std::cout<<std::endl;
    for(int i = 0; i<r->children.size(); i++) {
        print_tree(r->children[i]);
    }
}
*/
