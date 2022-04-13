# https://krpc.github.io/krpc/cpp/api/space-center/control.html

# The state of SAS.
bool sas

# The state of RCS.
bool rcs

# Returns whether all reactive wheels on the vessel are active.
bool reaction_wheels

# The state of the landing gear/legs.
bool gear

# Returns whether all landing legs on the vessel are deployed.
bool legs

# Returns whether all wheels on the vessel are deployed.
bool wheels

# The state of the lights.
bool lights

# The state of the wheel brakes.
bool brakes

# The state of the throttle.
# A value between 0 and 1.
float32 throttle

# The state of the pitch control.
# A value between -1 and 1. Equivalent to the w and s keys.
float32 pitch

# The state of the yaw control.
# A value between -1 and 1. Equivalent to the a and d keys.
float32 yaw

# The state of the roll control.
# A value between -1 and 1. Equivalent to the q and e keys.
float32 roll

# The state of the forward translational control.
# A value between -1 and 1. Equivalent to the h and n keys.
float32 forward

# The state of the up translational control.
# A value between -1 and 1. Equivalent to the i and k keys.
float32 up

# The state of the right translational control.
# A value between -1 and 1. Equivalent to the j and l keys.
float32 right
