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
    MotionTree() : nodes(1, Node(0)) {}  // Initialize the tree with a root node (id = 0
    std::vector<Node> nodes;    // Vector of nodes in the tree

    Node newVertex(StateSpace::VehicleState s_new);     // Add a new vertex to the tree based on a current state

    // These functions below shouldn't be necessary since they
    // can be accessed/set through the Node struct
    //void parent(Node& n_, int parent_);
    //void state(Node& n_, StateSpace::VehicleState s_);
    //void control(Node& n_, ControlSpace::VehicleControl m_);
    
};