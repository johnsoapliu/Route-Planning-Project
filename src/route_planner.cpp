#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    
    for(const auto neighbor : current_node->neighbors) {
        if (neighbor->visited) {
            continue;
        }
        neighbor->parent = current_node;
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->h_value = CalculateHValue(neighbor);        
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}


bool Compare(RouteModel::Node const *a, RouteModel::Node const *b){
    const float f1 = a->g_value + a->h_value;
    const float f2 = b->g_value + b->h_value;
    return f1 > f2;
}

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    
    auto next_node = open_list.back();
    open_list.pop_back();
    next_node->visited = true;
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        path_found.push_back(*current_node);
        auto next_parent = current_node->parent;
        if (next_parent) {
            distance += next_parent->distance(*current_node);
        }
        current_node = next_parent;
    }

    // Reverse the path_found list
    std::reverse(path_found.begin(), path_found.end());
    
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    open_list = {};
    open_list.push_back(start_node);

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node == end_node) {
            break;
        }
        this->AddNeighbors(current_node);
    }
    m_Model.path = ConstructFinalPath(end_node);
}