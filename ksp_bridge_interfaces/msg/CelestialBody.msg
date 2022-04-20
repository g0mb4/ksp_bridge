# https://krpc.github.io/krpc/cpp/api/space-center/celestial-body.html

# Note: no satellites, biomes nor orbit

string name
float32 mass
float32 gravitational_parameter
float32 surface_gravity
float32 rotational_period
float32 rotational_speed
float64 rotation_angle
float64 initial_rotation
float32 equatorial_radius
float32 sphere_of_influence
bool has_atmosphere
float32 atmosphere_depth
bool has_atmospheric_oxygen
float32 flying_high_altitude_threshold
float32 space_high_altitude_threshold
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Quaternion rotation
geometry_msgs/Vector3 direction
geometry_msgs/Vector3 angular_velocity
