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

        Node(){id = -1;};
        Node(int id) : id(id) {}
    };
    /**
     * Initialize the tree with a root node (id = 0)
    */
    MotionTree() : nodes(1, Node(0)) {}
    
    std::vector<Node> nodes;    // Vector of nodes in the tree

    /**
     * Add a new vertex to the tree based on a current state
     * @param s_new : VehicleState
     * @return new Node
    */
    Node newVertex(StateSpace::VehicleState s_new);    
    /**
     * Get a node from the tree based on its ID
     * @param id : ID of the node
     * @return Node
    */
    Node getNode(int id);             
    /**
     * Get the path from the root to a node
     * @param v_last : Node (last node in the path)
     * @return vector of Nodes (path)
    */                  
    std::vector<Node> getPath(Node v_last);       

    void addChild(int id_, int child_id){
        nodes[id_].children.push_back(child_id);
    }  

    void setParent(int id_, int parent_id){
        nodes[id_].parent = parent_id;
    }  
    
    void setRegion(int id_, int region_id){
        nodes[id_].region = region_id;
    }  

    void setControl(int id_, ControlSpace::VehicleControl control_){
        nodes[id_].control = control_;
    }
    
};