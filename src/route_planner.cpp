#include "route_planner.h"
#include <algorithm>

//==========================================================
// RoutePlanner Class
// --> Implements the A* Search Algorithm
//==========================================================

RoutePlanner::RoutePlanner(RouteModel &model,
                           float start_x, float start_y,
                           float end_x, float end_y) :
    m_Model(model)
{
    //Find pointers to closest nodes to start/end position
    //Note: coordinates are changed to percent
    start_node = &m_Model.FindClosestNode(start_x * 0.01, start_y * 0.01);
    end_node   = &m_Model.FindClosestNode(end_x * 0.01, end_y * 0.01);
}

//------------------------------
// Public methods
//------------------------------

// Search path from start_node to end_node

void RoutePlanner::AStarSearch()
{
    // Set start_node->visited to be true
    start_node->visited = true;

    // Initialize open_list with start_node
    open_list.push_back(start_node);

    // Create a pointer RouteModel::Node *current_node and set to nullptr
    RouteModel::Node *current_node = nullptr;

    // Expand nodes until the goal is reached. Use heuristic to prioritize
    // what node to open first
    while (open_list.size() > 0)
    {
        // Find the next best node to explore
        current_node = NextNode();

        // Check if the next best node is the goal (i.e. end point):
        if (current_node->distance(*end_node) == 0)
        {
            // Set the model path variable with the path found
            m_Model.path = ConstructFinalPath(current_node);

            // Return to exit the A* search
            return ;
        }

        // Add neighboring nodes to the list of nodes to explore
        AddNeighbors(current_node);
    }
}

// Calculate the h-value for a given node
// (h=value is computed as the euclidean distance from the node to the end node)
//
// Note: this method should be private. Made public so it jives with
// the unit testing provided by Udacity

float RoutePlanner::CalculateHValue(RouteModel::Node * node)
{
    return node->distance(*end_node);
}



//------------------------------
// Private methods
//------------------------------

//Rebuild path from last Node, from parent to parent until reaching start node

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath
                                            (RouteModel::Node *current_node)
{
    //empty path and 0 distance
    std::vector<RouteModel::Node> pathFound;
    distance = 0.f;

    //Add current node to begining of the path and update total distance
    while (current_node->parent != nullptr)
    {
        pathFound.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    pathFound.push_back(*current_node);

    //Apply scale to distance
    distance *= m_Model.MetricScale();

    return pathFound;
}


// Return the node with the lowest f-value,
// and remove the node from the list of open nodes

RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort the open_list according to the f-value,
    // which is the sum of a node's h-value and g-value.
    std::sort(open_list.begin(), open_list.end(),
                [](const auto &_1st, auto &_2nd) {
                    return ((_1st->g_value + _1st->h_value) <
                            (_2nd->g_value + _2nd->h_value));
                });

    // Create a copy of the pointer to the node with the lowest f-value.
    RouteModel::Node * lowest_f_value_node = open_list.front();

    // Erase that node pointer from open_list
    open_list.erase(open_list.begin());

    // Return the pointer copy
    return lowest_f_value_node;
}


// Add new nodes to the search

void RoutePlanner::AddNeighbors(RouteModel::Node * current_node)
{
    //  Populate the current_node's neighbors vector
    current_node->FindNeighbors();

    // For each neighbor in the current_node's neighbors
    for (auto neighbor : current_node->neighbors)
    {
        // Set the neighbor's parent to the current_node
        neighbor->parent = current_node;

        // Set the neighbor's g_value to the sum of the current_node's
        // g_value plus the distance from the curent_node to the neighbor
        neighbor->g_value = current_node->g_value
                            + current_node->distance(*neighbor);

        // Set the neighbor's h_value using CalculateHValue
        neighbor->h_value =  CalculateHValue(neighbor);

        // Push the neighbor to the back of the open_list
        open_list.push_back(neighbor);

        // Mark the neighbor as visited
        neighbor->visited = true;
    }
}
