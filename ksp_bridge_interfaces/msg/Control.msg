# https://krpc.github.io/krpc/cpp/api/space-center/control.html

uint8 SOURCE_KERBAL = 0
uint8 SOURCE_PROBE = 1
uint8 SOURCE_NONE = 3

uint8 STATE_FULL = 0
uint8 STATE_PARTIAL = 1
uint8 STATE_NONE = 2

uint8 SAS_MODE_STABILITY_ASSIST = 0
uint8 SAS_MODE_MANEUVER = 1
uint8 SAS_MODE_PROGRADE = 2
uint8 SAS_MODE_RETROGRADE = 3
uint8 SAS_MODE_NORMAL = 4
uint8 SAS_MODE_ANTI_NORMAL = 5
uint8 SAS_MODE_RADIAL = 6
uint8 SAS_MODE_ANTI_RADIAL = 7
uint8 SAS_MODE_TARGET = 8
uint8 SAS_MODE_ANTI_TARGET = 9

uint8 SPEED_MODE_ORBIT = 0
uint8 SPEED_MODE_SURFACE = 1
uint8 SPEED_MODE_TARGET = 2

uint8 CONTROL_INPUT_MODE_ADDITIVE = 0
uint8 CONTROL_INPUT_MODE_OVERRIDE = 1

std_msgs/Header header

uint8 source
uint8 state
bool sas
uint8 sas_mode
uint8 speed_mode
bool rcs
bool reaction_wheels
bool gear
bool legs
bool wheels
bool lights
bool brakes
bool antennas
bool cargo_bays
bool intakes
bool parachutes
bool radiators
bool resource_harvesters
bool resource_harvesters_active
bool solar_panels
bool abort
float32 throttle
uint8 input_mode
float32 pitch
float32 yaw
float32 roll
float32 forward
float32 up
float32 right
float32 wheel_throttle
float32 wheel_steering
int32 current_stage
