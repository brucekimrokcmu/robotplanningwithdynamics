#include <vector>

#include "VehicleState.h"
#include "VehicleMotion.h"

class MotionTree {
public:
    struct Node {
        int id;                     // Unique ID of the node
        int parent;                 // ID of parent
        int region;                 // Region ID of the node
        std::vector<int> children;  // IDs of the children nodes
        VehicleState state;         // state of Node
        VehicleMotion motion;       // Motion to the Node

        Node(int id) : id(id) {}
    };

    std::vector<Node> nodes;    // Vector of nodes in the tree

    Node newVertex();
    void parent(Node& n_);
    void state(Node& n_);
    void control(Node& n_);
    
};