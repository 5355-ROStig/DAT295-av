# Below is EG5355
#
# y x ->
# |
# V
#
# We use this notation for orientation
#    +-------+
#    |       |
#    |   N   |  windows
#    | W + E |
# door   S   |
#    +--------

# If you're standing on the road looking towards the intersection, the left
# position is the leftmost edge of the rectangle closest to the intersection.
#    |       |
#    |       |
#  right    left
# ---+       +----
#
Position left
Position right

# How far the road extends from the intersection
int32 length

# Road name (eg. "N", "W")
string name

