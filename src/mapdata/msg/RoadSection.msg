# If you're standing on the road looking towards the intersection, the left
# position is the leftmost edge of the rectangle closest to the intersection.
#    |       |
#    |       |
#  right    left
# ---+       +----
#
Position left
Position right

# How far the road section extends from the intersection
int32 length

# How far from the intersection the cars should stop
int32 stopline_offset
