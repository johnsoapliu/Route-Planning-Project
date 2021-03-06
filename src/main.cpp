#include <optional>
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <io2d.h>
#include "route_model.h"
#include "render.h"
#include "route_planner.h"

using namespace std::experimental;

static std::optional<std::vector<std::byte>> ReadFile(const std::string &path)
{   
    std::ifstream is{path, std::ios::binary | std::ios::ate};
    if( !is )
        return std::nullopt;
    
    auto size = is.tellg();
    std::vector<std::byte> contents(size);    
    
    is.seekg(0);
    is.read((char*)contents.data(), size);

    if( contents.empty() )
        return std::nullopt;
    return std::move(contents);
}

int main(int argc, const char **argv)
{    
    std::string osm_data_file = "";
    if( argc > 1 ) {
        for( int i = 1; i < argc; ++i )
            if( std::string_view{argv[i]} == "-f" && ++i < argc )
                osm_data_file = argv[i];
    }
    else {
        std::cout << "To specify a map file use the following format: " << std::endl;
        std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
        osm_data_file = "../map.osm";
    }
    
    std::vector<std::byte> osm_data;
 
    if( osm_data.empty() && !osm_data_file.empty() ) {
        std::cout << "Reading OpenStreetMap data from the following file: " <<  osm_data_file << std::endl;
        auto data = ReadFile(osm_data_file);
        if( !data )
            std::cout << "Failed to read." << std::endl;
        else
            osm_data = std::move(*data);
    }

    float start_x, start_y, end_x, end_y;
    float s_x, s_y, e_x, e_y;
    std::cout << "Input values from 0 to 100" << "\n";
    
    std::cout << "x coordinate of starting point: ";
    std::cin >> s_x;
    if (s_x <= 100 && s_x >= 0) {
        start_x = s_x;
    }
    else {
        std::cout << "Error: value is outside the limits" << "\n";
    }

    std::cout << "y coordinate of starting point: ";
    std::cin >> s_y;
    if (s_y <= 100 && s_y >= 0) {
        start_y = s_y;
    }
    else {
        std::cout << "Error: value is outside the limits" << "\n";
    }

    std::cout << "x coordinate of goal: ";
    std::cin >> e_x;
    if (e_x <= 100 && e_x >= 0) {
        end_x = e_x;
    }
    else {
        std::cout << "Error: value is outside the limits" << "\n";
    }

    std::cout << "y coordinate of goal: ";
    std::cin >> e_y;
    if (e_y <= 100 && e_y >= 0) {
        end_y = e_y;
    }
    else {
        std::cout << "Error: value is outside the limits" << "\n";
    }

    std::cout << "You have entered start point as: " << start_x << "," << start_y << "\n";
    std::cout << "You have entered goal as: " << end_x << "," << end_y << "\n";


    // Build Model.
    RouteModel model{osm_data};

    // Create RoutePlanner object and perform A* search.

    RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};

    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};

    auto display = io2d::output_surface{400, 400, io2d::format::argb32, io2d::scaling::none, io2d::refresh_style::fixed, 30};
    display.size_change_callback([](io2d::output_surface& surface){
        surface.dimensions(surface.display_dimensions());
    });
    display.draw_callback([&](io2d::output_surface& surface){
        render.Display(surface);
    });
    display.begin_show();
}
