#include "kdTree.h"

using namespace crpp;

template <class T, int N>
void kdTree<T, N>::insert(T newVal){
    if(this->root == NULL)
    {
        this->root = new kdTreeNode<T>(newVal);
    }
    else
    {
        return insertHelper(newVal, root, 0);
    }
}

template <class T, int N>
kdTreeNode<T> * kdTree<T, N>::findNearest (T newVal)
{
    if(root == NULL)
    {
        return NULL;
    }
    else
    {
        T lower, upper;
        for(int i = 0; i<N; i++)
        {
            lower[i] = -std::numeric_limits<double>::max();
            upper[i] = std::numeric_limits<double>::max();
        }
        return findNearestHelper(newVal, root, lower, upper, std::numeric_limits<double>::max(), 0);
    }

}

template <class T, int N>
kdTreeNode<T> * kdTree<T, N>::findNearestHelper (T newVal, kdTreeNode<T> * curNode, T lowerB, T upperB, double cur_min, int dps)
{
    double curDist = NdDist(newVal, curNode->val);
    if(lowerBoundInRange(newVal, lowerB, upperB) >= cur_min)
    {
        return NULL;
    }
    if(curNode->left == NULL && curNode->right == NULL)
    {
        if(sqrt(curDist) < cur_min)
        {
            return curNode;
        }
        else
        {
            return NULL;
        }
    }
    else
    {
        T lowerB1 = lowerB;
        lowerB1[dps%N] = curNode->val[dps%N];
        T upperB1 = upperB;
        upperB1[dps%N] = curNode->val[dps%N];
        double cur_min1 = std::min(sqrt(curDist), cur_min);
        if(curNode -> left == NULL)
        {
            kdTreeNode<T> * N1 = findNearestHelper (newVal, curNode->right, lowerB, upperB1, cur_min1, dps+1);
            return selectNN(newVal, N1, curNode);
        }
        else if(curNode -> right == NULL)
        {
            kdTreeNode<T> * N1 = findNearestHelper (newVal, curNode->right, lowerB1, upperB, cur_min1, dps+1);
            return selectNN(newVal, N1, curNode);
        }
        else
        {
            kdTreeNode<T> * N1 = findNearestHelper (newVal, curNode->right, lowerB, upperB1, cur_min1, dps+1);
            kdTreeNode<T> * N2 = findNearestHelper (newVal, curNode->right, lowerB1, upperB, cur_min1, dps+1);
            kdTreeNode<T> * N1N2 = selectNN(newVal, N1, N2);
            return selectNN(newVal, curNode, N1N2);
        }
    }
}

template <class T, int N>
void kdTree<T, N>::insertHelper(T newVal, kdTreeNode<T> * subTreeRoot, int dps)
{
    int cd = dps % N;
    if(subTreeRoot->val[cd] < newVal[cd])
    {
        if(subTreeRoot->right == NULL)
        {
            subTreeRoot->right = new kdTreeNode<T>(newVal);
        }
        else
        {
            insertHelper(newVal, subTreeRoot->right, dps+1);
        }
    }
    else
    {
        if(subTreeRoot->left == NULL)
        {
            subTreeRoot->left = new kdTreeNode<T>(newVal);
        }
        else
        {
            insertHelper(newVal, subTreeRoot->left, dps+1);
        }
    }
}

template <class T, int N>
double kdTree<T, N>::lowerBoundInRange(T newVal, T lower, T upper)
{
    T possibleNN;
    for(int i = 0; i<N; i++)
    {
        possibleNN[i] = (newVal[i]<lower[i])?(lower[i]):(newVal[i]>upper[i]?(upper[i]):(newVal[i]));
    }
    return NdDist(newVal, possibleNN);
}

template <class T, int N>
double kdTree<T, N>::NdDist(T N1, T N2)
{
    double res = 0;
    for(int i = 0; i<N; i++)
    {
        res += (N1[i]-N2[i]) * (N1[i]-N2[i]);
    }
    return sqrt(res);
}

template <class T, int N>
kdTreeNode<T> * kdTree<T, N>::selectNN(T newVal, kdTreeNode<T>* N1, kdTreeNode<T> * N2)
{
    if(N1 == NULL)
    {
        return N2;
    }
    else if(N2 == NULL)
    {
        return N1;
    }
    else if( NdDist(newVal, N1->val) < NdDist(newVal, N2->val) )
    {
        return N1;
    }
    else
    {
        return N2;
    }
}

template class kdTree<std::vector<double>, 2>;
