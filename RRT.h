#ifndef RRT_H
#define RRT_H

#include <vector>
#include <iostream>
#include <cmath>

namespace crpp{

    typedef struct treenode{
        std::vector<double> coordinate;
        treenode* parent;
        std::vector<treenode*> children;
        treenode(int dim) {
            this->coordinate = std::vector<double>(dim, 0);
            this->parent = NULL;
            this->children = std::vector<treenode *>();
        }
    }TreeNode;

    class RRT{
    private:
        int dim;
        double stepSize;
    public:
        TreeNode * root;
        RRT();
        RRT(std::vector<double> root_coord);
        TreeNode * nearestNode( TreeNode * curNode, TreeNode * target );
        double nodeDistance(const TreeNode* node1, const TreeNode* node2);
        bool expandTree(const std::vector<double>& target);
        bool expandFromNode(TreeNode * fromNode, TreeNode * tarNode);
        bool collision(const std::vector<double>& target);
        //void speak();
        //void print_tree(TreeNode* r);
        //void _deleteTree(TreeNode * tn);
        ~RRT();
    };
}

#endif /* end of include guard:  */
