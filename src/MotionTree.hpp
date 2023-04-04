#pragma once
#include <vector>

#include "StateSpace.hpp"
#include "ControlSpace.hpp"

class MotionTree {
public:
    struct Node {
        int id;                     // Unique ID of the node
        int parent;                 // ID of parent
        int region;                 // Region ID of the node
        std::vector<int> children;  // IDs of the children nodes
        StateSpace::VehicleState state;         // state of Node
        ControlSpace::VehicleControl control;       // Motion to the Node

        Node(int id) : id(id) {}
    };

    std::vector<Node> nodes;    // Vector of nodes in the tree

    Node NewVertex();
    void Parent(Node& n_, int parent_);
    void State(Node& n_, StateSpace::VehicleState s_);
    void Control(Node& n_, ControlSpace::VehicleControl m_);
    
};