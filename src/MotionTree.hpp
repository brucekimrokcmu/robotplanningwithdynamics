#pragma once
#include <vector>


template<typename State, typename Control>
class MotionTree {
public:
    struct Node {
        int id;                     // Unique ID of the node
        int parent;                 // ID of parent
        int region;                 // Region ID of the node
        std::vector<int> children;  // IDs of the children nodes
        State state;         // state of Node
        Control control;       // Motion to the Node

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
    Node newVertex(State s_new){
        Node n;
        n.id = nodes.size();
        n.state = s_new;
        nodes.push_back(n);
        return n;
    };    
    /**
     * Get a node from the tree based on its ID
     * @param id : ID of the node
     * @return Node
    */
    Node getNode(int id){
        return nodes[id];
    };             
    /**
     * Get the path from the root to a node
     * @param v_last : Node (last node in the path)
     * @return vector of Nodes (path)
    */                  
    std::vector<Node> getPath(Node v_last){
        std::vector<Node> path;
        Node n = v_last;
        while(n.id != 0){
            path.push_back(n);
            n = nodes[n.parent];
        }
        path.push_back(n);
        return path;
    };       

    void addChild(int id_, int child_id){
        nodes[id_].children.push_back(child_id);
    }  

    void setParent(int id_, int parent_id){
        nodes[id_].parent = parent_id;
    }  
    
    void setRegion(int id_, int region_id){
        nodes[id_].region = region_id;
    }  

    void setControl(int id_, Control control_){
        nodes[id_].control = control_;
    }
    
};