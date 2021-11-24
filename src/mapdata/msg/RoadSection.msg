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

# Enumeration (just constants) of priority signs
uint8 PRIORITY_ROAD=0
uint8 GIVE_WAY=1
uint8 STOP_SIGN=2
uint8 TRAFFIC_LIGHT=3

unit8 priority_sign

# A bit redundant but nice for pretty printing
string name
