#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

//==========================================================
// RouteModel Class
// --> Expand the OSM Model class to Enable the A* Search Algorithm
//     implemented by the RoutePlanner class
//==========================================================

class RouteModel : public Model
{
  public:

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Node Class
    // --> Extend the Model class (model.h) to enable the A* Search Algorithm
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    class Node : public Model::Node
    {
      public:
        // Previous node in path
        RouteModel::Node * parent = nullptr;

        // Heuristic value
        float h_value = std::numeric_limits<float>::max();

        // Cost from starting point
        float g_value = 0.0;

        // A* Search visited node
        bool visited = false;

        // List of adjacent nodes
        std::vector<Node *> neighbors;

        // Class constructors
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

        // Populate neighbors vector for this node
        // --> neighbors are closest nodes on all roads this node is on
        void FindNeighbors() ;

        // Euclidian distance between nodes
        float distance (Node other) const
        {
          return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
        }

      private:
        // Node index in the m_Nodes list
        int index;

        // Parent model
        RouteModel * parent_model = nullptr;

        // Find closest node from a list of nodes
        Node * FindNeighbor(std::vector<int> node_indices);
    };

    // The path that is found by the A* search
    std::vector<Node> path;

    // RouteModel class constructor
    RouteModel(const std::vector<std::byte> &xml);

    // Find node in OSM model which is closest to user provided cooordinates
    Node &FindClosestNode(float x, float y);

    // Getter for NodeToRoad hashmap
    auto &GetNodeToRoadMap() { return node_to_road; }

    // Getter for the private nodes vector
    auto &SNodes() { return m_Nodes; }

  private:
    // Map for NodeID -> Road, used by A* Search
    std::unordered_map<int, std::vector<const Model::Road*>> node_to_road;

    // A* enabled Nodes
    std::vector<Node> m_Nodes;

    // Build the NodeToRoad hashmap
    void CreateNodeToRoadHashmap();
};
