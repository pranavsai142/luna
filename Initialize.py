import datetime
import numpy as np

import Constants as const
import OrbitalMath as orbitm
import VectorMath as vecm

import Simulation
import ConditionNode
import Rocket

init_latitude = np.radians(44.91)
init_longitude = np.radians(-92.31)
init_height = 397.507 * 1000
radiusFromEarth = orbitm.find_radius_from_body(
	init_latitude, 
	const.EARTH_A_RADIUS, 
	const.EARTH_FLATTEN_CONST, 
	init_height)
init_perigee = radiusFromEarth
init_apogee = radiusFromEarth
YEAR = 1995
MONTH = 11
DAY = 18
HOUR = 12
MINUTE = 46
SECOND = 0
inclination = np.radians(44.91)

init_mass = 10
init_dry_mass = 8.75
specific_impulse = 360
mass_flux = .1
delta_theta = .1
secondsOfRCS = datetime.timedelta(seconds = 10000)

semimajor = orbitm.find_semimajor(init_apogee, init_perigee)
print("Initial radiusFromEarth", radiusFromEarth)
print("Initial semimajor", semimajor)
print("init_perigee", init_perigee)
velocity = orbitm.find_velocity_at_point(init_perigee, semimajor, const.EARTH_STANDARD_GRAV)
print("velocity", velocity)
init_datetimeUTC = datetime.datetime(year=YEAR, month=MONTH, day=DAY, hour=HOUR, minute=MINUTE, second=SECOND)


initPos = orbitm.to_eci_coordinates(
	init_latitude, 
	init_longitude, 
	init_height, 
	init_datetimeUTC, 
	const.EARTH_FLATTEN_CONST, 
	const.EARTH_A_RADIUS)



initVel = orbitm.extrapolate_velocity_vector_at_periapsis(initPos, inclination, velocity)
initOrient = vecm.unit_vector(initPos)
# 	initOrient = vecm.unit_vector(vecm.vector(1,90,20))
initEccentricity = orbitm.find_eccentricity(initPos, initVel, const.EARTH_STANDARD_GRAV)
initFlightAngle = orbitm.find_flight_angle_state_vectors(initPos, initVel)

print("apoperii", orbitm.find_apo_peri(initEccentricity, semimajor))

condition_nodes = []

for line in open("FLIGHT_MANIFEST.txt"):
	li=line.strip()
	if not li.startswith("#"):
		condition_nodes.append(ConditionNode.ConditionNode(line.rstrip()))
		
		
maneuver_nodes = []

initRocket = Rocket.Rocket(
	init_datetimeUTC,
	initPos, 
	initVel, 
	init_mass, 
	init_dry_mass,
	initOrient, 
	specific_impulse, 
	mass_flux,
	delta_theta,
	secondsOfRCS,
	maneuver_nodes)

# condition_nodes[1].createManeuverList(initRocket)

print("initPos", initPos)
print("initVel", initVel)
print("initVelMag", vecm.mag(initVel))
print("visVisa vel", velocity)
print("initEccentricity", initEccentricity)
print("initFlightAngle", initFlightAngle)

Simulation.startSimulation(initRocket)

# 	planets = load('de421.bsp')
# 	earth = planets['earth']
# 	
	
	
