#!/usr/bin/env python3
"""
Uses the previously generated graph data (from '17 or '18, who knows...) to generate a GraphML graph instead.

This should basically only be used once, just to convert the previous graph into GraphML, without having to
re-draw all 600+ elements of the original graph.

New maps shouldn't be created with this at all, use a real GraphML editor instead.

Code for parsing this custom format inspired by the original graph_func.py in
the truck_map repo.

Run like this to convert the original map data to GraphML:
    python3 graph_converter.py graph.txt eg5355.xml
This assumes the graph.txt file is in this directory.
This will overwrite the eg5355.xml output file if it exists!

Copyright (c) 2020, Thomas Alexandersson
Copyright (c) 2020, Farzad Besharati
Copyright (c) 2020, Johannes Gustavsson
Copyright (c) 2020, Martin Hilgendorf
Copyright (c) 2020, Ivan Lyesnukhin
Copyright (c) 2020, Jian Shin
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Chalmers University of Technology nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import sys
from pathlib import Path
from typing import Tuple, Optional

import networkx as nx


def load_graph(graph_file_path: str) -> nx.DiGraph:
    graph_file = Path(graph_file_path)
    if not graph_file.is_file():
        raise FileNotFoundError("Could not find graph file")

    node_id_counter = 0
    edge_id_counter = 0

    # Node creation helpers
    current_node = None
    node_begin = False  # Used for syntax validation

    node_position_id_map = {}
    edge_id_map = {}

    with graph_file.open() as file:
        for line_num, line in enumerate(file):

            # Remove whitespace from line
            line: str = line.strip()

            # Skip blank lines and comments
            if not line or line.startswith('#'):
                continue

            if line == "NODE":
                if not node_begin:
                    node_begin = True
                else:
                    print(f"Syntax error on line {line_num} in '{graph_file.name}'")
            elif line == "ENDNODE":
                if node_begin and current_node:
                    node_begin = False
                    current_node = None
                else:
                    print(f"Syntax error on line {line_num} in '{graph_file.name}'")
            elif line[0].isdigit():
                if not node_begin:
                    print(f"Syntax error on line {line_num} in '{graph_file.name}'")
                parts = line.split(';')

                # If we are not currently writing a node, start a new one
                if not current_node and len(parts) == 1:
                    x, y = parse_coord(parts[0])

                    if (x, y) not in node_position_id_map:
                        node_position_id_map[(x, y)] = f"n{node_id_counter}"
                        node_id_counter += 1
                    current_node = (x, y)
                # If we are currently "in a node", these pieces of data are out-edges
                elif current_node:
                    for coord in parts:
                        x, y = parse_coord(coord)

                        # If the connected node is new, create it
                        if (x, y) not in node_position_id_map:
                            node_position_id_map[(x, y)] = f"n{node_id_counter}"
                            node_id_counter += 1

                        # Weight is the time it takes to traverse this node for CBS/SIPP
                        src_node_name = node_position_id_map[current_node]
                        dest_node_name = node_position_id_map[(x, y)]
                        edge_id_map[(src_node_name, dest_node_name)] = f"e{edge_id_counter}"
                        edge_id_counter += 1
            else:
                print(f"Syntax error on line {line_num} in '{graph_file.name}'")

    # Create the actual graph
    graph = nx.MultiDiGraph()

    # Add all nodes to the graph
    for pos, node_name in node_position_id_map.items():
        if node_name in graph.nodes():
            print(f"Trying to add node '{node_name}' twice")
            break
        graph.add_node(node_name, x=pos[0], y=pos[1])

    # Add all edges to the graph
    for (src_node, dest_node), edge_name in edge_id_map.items():
        if edge_name in graph.edges():
            print(f"Trying to add edge '{edge_name}' twice")
            break
        graph.add_edge(src_node, dest_node, key=edge_name, weight=1)

    return graph


def parse_coord(input_string: str) -> Optional[Tuple[float, float]]:
    values = input_string.split(',')
    if not len(values) == 2:
        return None
    x, y = map(int, values)

    return x, y


if __name__ == '__main__':
    if not len(sys.argv) == 3:
        print("Invalid number of arguments. Run like:\n    graph_converter.py input_file output_file\n")
        sys.exit(1)

    g = load_graph(sys.argv[1])

    # Draw the graph with matplotlib (layout is pretty trash though)
    # import matplotlib.pyplot as plt
    # nx.draw_networkx(g, with_labels = True)
    # plt.show()

    for line in nx.generate_graphml(g):
        print(line)

    nx.write_graphml(g, sys.argv[2])
