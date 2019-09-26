#pragma once

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"

//==========================================================
// RoutePlanner Class
// --> Implements the A* Search Algorithm
//==========================================================

class RoutePlanner {
  public:
    // RoutePlanner class constructor
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);

    //Get found solution distance
    float GetDistance() const { return distance; }

    // Search path from start_node to end_node
    void AStarSearch();

    //Rebuild path from last Node, from parent to parent until reaching start node
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);

    // Calculate the h-value for a given node
    float CalculateHValue(RouteModel::Node *);

    // Add new nodes to the search
    void AddNeighbors(RouteModel::Node *);

  private:
    // OSM model augmented to performed A*
    RouteModel & m_Model;

    //Start and end Nodes of the search
    RouteModel::Node *start_node, *end_node;

    //Found route distance
    float distance;

    // the list of open nodes in the A* search
    std::vector<RouteModel::Node *> open_list;

    // Return the node with the lowest f-value,
    // and remove the node from the list of open nodes
    RouteModel::Node *NextNode();
};
