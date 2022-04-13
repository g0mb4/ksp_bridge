# https://krpc.github.io/krpc/cpp/api/space-center/flight.html

# Name of the vessel
string name

# The total mass of the vessel, including resources, in kg.
float32 mass

# The total mass of the vessel, excluding resources, in kg.
float32 dry_mass

#Flight data

# The current G force acting on the vessel in g.
float32 g_force

# The altitude above sea level, in meters.
# Measured from the center of mass of the vessel.
float64 mean_altitude

# The altitude above the surface of the body or sea level, whichever is closer, in meters.
# Measured from the center of mass of the vessel.
float64 surface_altitude

# The altitude above the surface of the body, in meters.When over water, this is the altitude above the sea floor.
# Measured from the center of mass of the vessel.
float64 bedrock_altitude

# The elevation of the terrain under the vessel, in meters.This is the height of the terrain above sea level,
# and is negative when the vessel is over the sea.
float64 elevation

# The latitude of the vessel for the body being orbited, in degrees.
float64 latitude

# The longitude of the vessel for the body being orbited, in degrees.
float64 longitude
