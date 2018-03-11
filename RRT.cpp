#include "RRT.h"

using namespace crpp;

RRT::RRT() {
    this->dim = 2;
    this->stepSize = 0.1;
}

RRT::RRT(std::vector<double> root_coord) {
    this->dim = 2;
    this->stepSize = 0.1;
    this->root = new TreeNode(this->dim);
    this->root->coordinate = root_coord;
}


RRT::~RRT() {}

TreeNode * RRT::nearestNode( TreeNode * curNode, TreeNode * target ){
    if(curNode == NULL) {
        return NULL;
    }
    double curRec = nodeDistance( curNode, target );
    TreeNode * nodeRec = curNode;
    for(int i = 0; i<curNode->children.size(); i++) {
        TreeNode * tmpNode = nearestNode( curNode->children[i], target );
        double curDist = nodeDistance( tmpNode, target );
        if(curDist < curRec) {
            curRec = curDist;
            nodeRec = tmpNode;
        }
    }
    return nodeRec;
}

bool RRT::expandTree(const std::vector<double>& target) {
    // find nearest node
    TreeNode * tarNode = new TreeNode(this->dim);
    tarNode->coordinate = target;
    TreeNode * nearest = nearestNode(this->root, tarNode);
    return expandFromNode(nearest, tarNode);
}

bool RRT::expandFromNode(TreeNode * fromNode, TreeNode * tarNode) {
    double totalDist = nodeDistance(fromNode, tarNode);
    if(totalDist < this->stepSize) {
        tarNode->parent = fromNode;
        fromNode->children.push_back(tarNode);
        return true;
    } else {
        TreeNode * tmpNode = new TreeNode(this->dim);
        for(int i = 0; i<tarNode->coordinate.size(); i++)
        {
            tmpNode->coordinate[i] = fromNode->coordinate[i] + (tarNode->coordinate[i] - fromNode->coordinate[i]) * this->stepSize / totalDist;
        }
        if(this->collision(tmpNode->coordinate)) {
            return false;
        } else {
            tmpNode->parent = fromNode;
            fromNode->children.push_back(tmpNode);
            return expandFromNode(tmpNode, tarNode);
        }
    }
}

bool RRT::collision(const std::vector<double>& target) {
    return false;
}

double RRT::nodeDistance(const TreeNode* node1, const TreeNode* node2) {
    double dst = 0;
    for(int i = 0; i<node1->coordinate.size(); i++)
    {
        dst += (node1->coordinate[i] - node2->coordinate[i]) * (node1->coordinate[i] - node2->coordinate[i]);
    }
    return std::sqrt(dst);
}

/*
void RRT::speak() {
    print_tree(this->root);
}

void RRT::print_tree(TreeNode * r) {
    for(int i = 0; i<this->dim; i++) {
        std::cout<<r->coordinate[i]<<" ";
    }
    std::cout<<std::endl;
    for(int i = 0; i<r->children.size(); i++) {
        print_tree(r->children[i]);
    }
}
*/
