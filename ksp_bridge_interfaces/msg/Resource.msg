# https://krpc.github.io/krpc/cpp/api/space-center/resources.html

uint8 FLOW_MODE_VESSEL = 0
uint8 FLOW_MODE_STAGE = 1
uint8 FLOW_MODE_ADJACENT = 2
uint8 FLOW_MODE_NONE = 3

string name
float32 amount
float32 max
float32 density
uint8 flow_mode
bool enabled
