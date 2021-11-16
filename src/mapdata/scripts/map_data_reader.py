"""
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
from pathlib import Path
from typing import Dict

import yaml

from environment import Map

MAP_DATA_DIR = Path(__file__).parent / '../maps'


def load_map(map_name: str) -> Map:
    """
    Loads a specific map given its name.

    The name is the name of the directory where the map is stored.
    """
    maps = find_existing_maps(MAP_DATA_DIR)

    if map_name not in maps:
        raise FileNotFoundError(f"Could not find requested map: '{map_name}'")

    map_data_dir = maps[map_name]

    config_file = map_data_dir / 'config.yml'
    config = read_config(config_file)

    return Map(map_name, map_data_dir / config['image-file'], config['image-scale'],
               map_data_dir / config['graph-file'], map_data_dir / config['cdm-data'])


def read_config(config_file: Path) -> Dict:
    """Reads a YAML map config file at a specific location"""
    if not config_file.is_file():
        raise FileNotFoundError(f"Could not find config file for map at: '{config_file.parent.resolve()}'")

    with config_file.open() as file:
        config = yaml.safe_load(file)

    return config


def find_existing_maps(data_dir: Path) -> Dict[str, Path]:
    """
    Finds all maps in the given map data directory of this repo

    Returns a mapping of
        map_name: data directory
    for each map.
    """
    # Check if map data directory exists
    if not data_dir.exists():
        raise FileNotFoundError(f"Could not find map data directory: '{data_dir}'")

    # Check if there are any subdirectories in the data directory
    map_paths = (child for child in data_dir.iterdir() if child.is_dir())
    if not map_paths:
        raise FileNotFoundError(f"Could not find any maps in map data directory")

    maps = {
        map_path.name: map_path for map_path in map_paths
    }

    return maps


if __name__ == '__main__':
    eg = load_map('eg5355')
    eg.get_occupancy_grid()
