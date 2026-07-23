# Semantic type of this cleaning-plan segment.
uint8 EDGE=1
uint8 COVERAGE=2
uint8 TRANSITION=3

uint8 type
uint32 component_index
# 0 is the outer boundary; positive values identify exclusion boundaries.
# -1 means the field does not apply.
int32 boundary_index
# Fields2Cover swath index, or -1 when the field does not apply.
int32 swath_index
geometry_msgs/PoseStamped start
geometry_msgs/PoseStamped end
