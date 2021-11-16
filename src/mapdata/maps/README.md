Mapdata maps
============

This directory contains maps that can be used with the system.

Each map is placed in its own directory (with the same name as the map).

Each map directory should contain the following data files:
 - `config.yml`: A config file specifying which files contain which data about the map environment
 - `critical_sections.yml`: Defines all critical sections for the CDM system
 - `road_graph.xml`: A GraphML graph of the road network
 - `map_image.png`: A top-down perspective view of the map. Black pixels denote offroad/blocked, white pixels are the 
 road, etc. See the ROS wiki about occupancy grids as well.  
