#ifndef RRT_H
#define RRT_H

#include "kdTree.h"
#include <cmath>
#include <iostream>
#include <vector>

namespace crpp {

template <class T, int N> class RRT : public kdTree<T, N> {
private:
  double stepSize;

public:
  RRT();
  RRT(T root_coord);
  double nodeDistance(const kdTreeNode<T> *node1, const kdTreeNode<T> *node2);
  bool expandTree(T target);
  bool expandFromNode(kdTreeNode<T> *fromNode, kdTreeNode<T> *tarNode);
  bool collision(T target);
};
} // namespace crpp

#endif /* end of include guard:  */
