# Visualizer

Subscribes to GulliView coordinates to display the positions of wifibots as markers in RViz. The markers are published to `/visualization_marker`

Assumes
- `mapdata` has published some map with a frame called 'map'
- a `gv_client` is posting to the topic /gv_positions

To start:

    rosrun visualization marker_viz.py