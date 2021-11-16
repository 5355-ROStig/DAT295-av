# Type enumeration
uint8 THREE_WAY_INTERSECTION=0
uint8 FOUR_WAY_INTERSECTION=1
uint8 THREE_WAY_ROUNDABOUT=2

# Orientation enumeration
uint8 NORTH=0
uint8 SOUTH=1
uint8 EAST=2
uint8 WEST=3


int32 id
Position pos
int32 type
int32 width
int32 height
int32 orientation
CDMConnectionZone[] connection_zones
