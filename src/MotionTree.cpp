#include "MotionTree.hpp"

MotionTree::Node MotionTree::newVertex(StateSpace::VehicleState s_new){
    Node n;
    n.id = nodes.size();
    n.state = s_new;
    nodes.push_back(n);
    return n;
}

MotionTree::Node MotionTree::getNode(int id){
    return nodes[id];
}

std::vector<MotionTree::Node> MotionTree::getPath(Node v_last){
    std::vector<Node> path;
    Node n = v_last;
    while(n.id != 0){
        path.push_back(n);
        n = nodes[n.parent];
        // if(n.parent >= nodes.size()){
        //     printf("Hree");
        // }
        printf("Hree");
    }
    path.push_back(n);
    return path;
}