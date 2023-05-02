#include "MotionTree.hpp"

template<typename State, typename Control>
typename MotionTree<State,Control>::Node MotionTree<State,Control>::newVertex(State s_new){
    Node n;
    n.id = nodes.size();
    n.state = s_new;
    nodes.push_back(n);
    return n;
}
template<typename State, typename Control>
typename MotionTree<State,Control>::Node MotionTree<State,Control>::getNode(int id){
    return nodes[id];
}
template<typename State, typename Control>
std::vector<typename MotionTree<State,Control>::Node> MotionTree<State,Control>::getPath(Node v_last){
    std::vector<Node> path;
    Node n = v_last;
    while(n.id != 0){
        path.push_back(n);
        n = nodes[n.parent];
    }
    path.push_back(n);
    return path;
}