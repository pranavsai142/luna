import numpy as np
import datetime
import VectorMath as vecm
import julian
import Constants as const

def find_velocity_at_point(radius, semimajor, STANDARD_GRAV):
	return np.sqrt(STANDARD_GRAV*((2/radius) - (1/semimajor)))
	
# def find_eccentricity(radius, velocity, flight_angle, STANDARD_GRAV):
# 	part = ((((radius * (velocity**2)) / STANDARD_GRAV) - 1)**2) * (m.cos(flight_angle))**2
# 	return m.sqrt(part + (m.sin(flight_angle)**2))
	
def find_eccentricity(pos, vel, STANDARD_GRAV):
	return vecm.mag(find_eccentricity_vector(pos, vel, STANDARD_GRAV))

def find_eccentricity_vector(pos, vel, STANDARD_GRAV):
	orbitalPlaneNormal = vecm.cross_vector(pos, vel)
	a = vecm.cross_vector(vel, orbitalPlaneNormal) / STANDARD_GRAV
	b = pos / vecm.mag(pos)
	eVec = a - b
	return eVec
	
def find_eccentric_anomaly(eccentricity, trueAnomaly):
	a = np.cos(trueAnomaly)
	b = (eccentricity + a) / (1 + (eccentricity*a))
	eccentricAnomaly = np.arccos(b)
	return eccentricAnomaly
	
def find_eccentricity_from_apo(semimajor, apo):
	return apo/semimajor - 1
	
def find_eccentricity_from_peri(semimajor, peri):
	return -peri/semimajor + 1
	
	
def find_semimajorly(radius, velocity, STANDARD_GRAV):
	return 1/(2/radius - velocity**2/STANDARD_GRAV)
	
def find_apo_peri(eccentricity, semimajor):
	peri = semimajor*(1-eccentricity)
	apo = semimajor*(1+eccentricity)
	return(apo, peri)
		
	
def find_max_flight_angle(eccentricity):
	max_flight_angle = np.asin(eccentricity)
	return max_flight_angle

def find_flight_angle(radius, semimajor, eccentricity):
	angle = radius**2 + (2*semimajor - radius) **2 - 4 * semimajor**2 * eccentricity**2
	result = angle/(2*radius * (2*semimajor - radius))
	flightAngle = np.acos(result)/2
	return flightAngle
	
def find_flight_angle_state_vectors(pos, vel):
	a = vecm.dot(pos, vel)
	b = vecm.mag(pos) * vecm.mag(vel)
	return np.arccos(a/b)

# def find_time_anomalies(radius, semimajor, eccentricity, STANDARD_GRAV):
# 	if(radius <= find_apo_peri(eccentricity, semimajor)[1] or eccentricity <= 0 or eccentricity >= 1):
# 		return (0,0,0)
# 	else:
# 		orbitalPeriod = 2 * np.pi * np.sqrt((semimajor ** 3) / STANDARD_GRAV)
# 		a = (semimajor - radius)/(eccentricity * semimajor)
# 		if(a < -1 or a > 1):
# 			print('a',a)
# 			print("semi,rad,ecc", semimajor, radius, eccentricity)
# 			return (0,0,0)
# 		else:
# 			b = np.sqrt((semimajor ** 3)/STANDARD_GRAV) 
# 			c = np.arccos(a)
# 			d = np.sqrt((eccentricity ** 2) - ((1 - (radius / semimajor)) ** 2))
# 			timeSincePeriapsis = b * (c - d)
# 			timeToPeriapsis = orbitalPeriod - timeSincePeriapsis
# 			return (orbitalPeriod, timeSincePeriapsis, timeToPeriapsis)
# 		

def find_time_anomalies(semimajor, trueanomaly, STANDARD_GRAV):
	orbitalPeriod = 0.0
	timeSincePeriapsis = 0.0
	timeToPeriapsis = 0.0
	if(semimajor > 0 and trueanomaly > 0):
		orbitalPeriod = 2 * np.pi * np.sqrt((semimajor ** 3) / STANDARD_GRAV)
		secondsSincePeriapsis = (trueanomaly / (2 * np.pi)) * orbitalPeriod
		secondsToPeriapsis = orbitalPeriod - secondsSincePeriapsis
		timeSincePeriapsis = datetime.timedelta(seconds = secondsSincePeriapsis)
		timeToPeriapsis = datetime.timedelta(seconds = secondsToPeriapsis)
	return (orbitalPeriod, timeSincePeriapsis, timeToPeriapsis)
		
	
def find_orbital_period(semimajor, STANDARD_GRAV):
	return 2 * np.pi * np.sqrt((semimajor ** 3) / STANDARD_GRAV)

def find_semimajor(apo, peri):
	return (apo + peri) / 2

def extrapolate_velocity_vector_at_periapsis(positionVec, inclination, velocity):
# 	Turn unit vector into position vec
	unitPositionVec = vecm.unit_vector(positionVec)
# 	print("unit pos vec", unitPositionVec)
# 	Define z axis
	zAxis = vecm.vector(0,0,1) 
	xAxis = vecm.vector(1,0,0)
# 	Create rotation vector to rotate z axis around and turn it into unit
	rotationVector = vecm.unit_vector(vecm.cross_vector(zAxis, unitPositionVec))
# 	print("rotationVector", rotationVector)
# 	print("magRotationVector", mag(rotationVector))
# 	Find angle of inclination based on vectors
	vecInclination = ((np.pi/2) - vecm.angleBetweenVectors(unitPositionVec, zAxis)) * -1
# 	print("vecInclination", vecInclination)
# 	Find normal vector of orbital plane by rotating z axis about rotationVector by vectorInclination
	orbitalPlaneNormal = vecm.rotate_around_arbitrary(zAxis, rotationVector, vecInclination)
	print("orbitalPlaneNormal", orbitalPlaneNormal)
	
# 	print("orbitalPlaneNormal", orbitalPlaneNormal)
# 	print("magOrbitalPlaneNormal", mag(orbitalPlaneNormal))
# 	Create velocity vector by scaling up rotationVector by magnitude of velocity.
	return(velocity * rotationVector)

def extrapolate_inclination(pos, vel):
	orbitalPlaneNormal = vecm.cross_vector(vecm.unit_vector(vel), vecm.unit_vector(pos))
	return np.arccos((orbitalPlaneNormal[2])/vecm.mag(orbitalPlaneNormal))
	
def find_true_anomaly(pos, vel, eccentricityVector):
	a = vecm.dot(eccentricityVector, pos)
	b = vecm.mag(eccentricityVector) * vecm.mag(pos)
	c = a/b
	trueAnomaly = 0
	if(abs(c) <= 1):
		trueAnomaly = np.arccos(c)
		if(vecm.dot(pos,vel) < 0):
			trueAnomaly = (2 * np.pi) - trueAnomaly
	return trueAnomaly
	
def find_eccentricity_from_periapsis(position, velocity, STANDARD_GRAV):
	return ((position* velocity**2)/STANDARD_GRAV)-1
	
def find_radius_from_body(latitude, BODY_A_RADIUS, FLATTEN_CONST, height):
	c = 1/np.sqrt(1+(FLATTEN_CONST * (FLATTEN_CONST - 2) * np.sin(latitude)**2))
	return ((BODY_A_RADIUS * c) + height)
	
def hhmmss_to_rad(hour, min, sec):
	return hour*15 + min/4 + (sec/240) * (np.pi/180)
def hhmmss_to_sec(hour, min, sec, microsec):
	return (hour * 3600) + (min * 60) + sec + (microsec * 1e-6)
	
# 	A function that takes in a date and finds the siderial time at greenwich
def find_greenwich_siderial_time(date):
# Isolata variables from passed date time
	year = date.year
	month = date.month
	day = date.day
	hour = date.hour
	min = date.minute
	sec = date.second
	microsec = date.microsecond
# 	Create date time object without time parts
	noTimeDate = datetime.datetime(year = year, month = month, day=day)
# 	Define J2000 julian date
	J2000JD = 2451545.0
# 	Convert stripped date to julian
	dateJD = julian.to_jd(noTimeDate)
# 	Find difference since J2000 epoch. result is in days
	deltaDays = (dateJD - J2000JD)
# 	print(deltaDays)

# Begin theta calculation for greenwich at 0 hours
	tU = deltaDays/36525
	a = 24110.54841
	b = 8640184.812866 * tU
	c = 0.093104 * tU**2
	d = -6.2e-6 * tU**3
	thetaGrenwich = a + b + c + d
# 	Convert to radians
	thetaGrenwich = hhmmss_to_rad(0, 0, thetaGrenwich)
	
# 	Find given UTC time in seconds
	deltaT = hhmmss_to_sec(hour, min, sec, microsec)
	
# 	Add in the accounting for the current time. Mod the value to find the equivalent angle
# All calculations here are in radians. DeltaT is in seconds but the seconds cancells with teh
# angular velocity which is in rad/sec
	radians = (thetaGrenwich + 7.29211510e-5 * deltaT)
	radians = radians % hhmmss_to_rad(0, 0, datetime.timedelta(days=1).total_seconds())
	return radians
	
# Find earth centered inertial coordinates at a given lat and long at a certain datetime
def to_eci_coordinates(latitude_rad, longitude_rad, height, datetimeUTC, FLATTEN_CONST, BODY_A_RADIUS):
# c and s constants to be used in calculations
	c = 1/np.sqrt(1+(FLATTEN_CONST * (FLATTEN_CONST - 2) * np.sin(latitude_rad)**2))
	s = (1-FLATTEN_CONST)**2 * c
	
# 	find theta value. Add in longitude rad to greenwich longitude_rad to determine current
# longitude sidereal time in radians
	theta = find_greenwich_siderial_time(datetimeUTC) + longitude_rad
	
# 	Calculate x, y, and z positions projected on to the celestial sphere based on the J2000 frame
	r = ((BODY_A_RADIUS * c) + height) * np.cos(latitude_rad)
	x = r * np.cos(theta)
	y = r * np.sin(theta)
	z = ((BODY_A_RADIUS * s) + height) * np.sin(latitude_rad)
	return vecm.vector(x,y,z)
	
def to_lat_long_height_from_eci(x, y, z, date, FLATTEN_CONST, BODY_A_RADIUS):
	latitude = np.arctan(z/np.sqrt(x**2 + y**2))
	greenwich_siderial_time = find_greenwich_siderial_time(date)
	
	arctan = np.arctan(y/x)
# 	Thank you MITRE corporation for these conditionals when doing the arctan to find the longitude
	if(x >= 0):
		arctan = arctan
	elif(x < 0 and y >= 0):
		arctan = np.pi + arctan
	elif(x < 0 and y < 0):	
		arctan = -np.pi + arctan
	longitude = arctan - greenwich_siderial_time
# 	Normalization equation to put longitude in [-180, 180] range
	longitude = (longitude + np.pi) % (2 * np.pi) - np.pi

	e_squared = (2 * FLATTEN_CONST) - (FLATTEN_CONST**2)
	rho = np.sqrt(x**2 + y**2 + z**2)
		
	subpoint_latitude = latitude
	r=(rho*np.cos(subpoint_latitude))
	c = 0
# 	Tolerance is in radians. Equivilant to 0.000002 degrees
	tolerance = 3.49066e-8
	while True:
		c = 1/np.sqrt(1 - ((e_squared) * (np.sin(latitude)**2)))
		subpoint_latitude = np.arctan((z + ((BODY_A_RADIUS * c * e_squared) * np.sin(latitude)))/r)
		if(abs(latitude - subpoint_latitude) <= tolerance):
			break
		else:
			latitude = subpoint_latitude
	
	height = (r / np.cos(subpoint_latitude)) - (BODY_A_RADIUS * c)
	return vecm.vector(subpoint_latitude, longitude, height)
	
def find_moon_lat_long(vectorFromMoon):
	latitude = np.arctan(vectorFromMoon[2] / np.sqrt(vectorFromMoon[0]**2 + vectorFromMoon[1]**2))
	arctan = np.arctan(vectorFromMoon[1]/vectorFromMoon[0])
# 	Thank you MITRE corporation for these conditionals when doing the arctan to find the longitude
	if(vectorFromMoon[0] >= 0):
		arctan = arctan
	elif(vectorFromMoon[0] < 0 and vectorFromMoon[1] >= 0):
		arctan = np.pi + arctan
	elif(vectorFromMoon[0] < 0 and vectorFromMoon[1] < 0):	
		arctan = -np.pi + arctan
	longitude = arctan
	return((latitude, longitude))
	
def getMoonLocation(year, month, day, hour, minute, second, microsecond):
# 	J2000 coordinates of moon at tick time
	return const.earth.at(const.ts.utc(year, month, day, hour, minute, (second + (microsecond*1e-6)))).observe(const.moon).position.m

def getMoonVelocity(year, month, day, hour, minute, second, microsecond):	
# 	J2000 velocity of moon at tick time
	return const.earth.at(const.ts.utc(year, month, day, hour, minute, (second + (microsecond*1e-6)))).observe(const.moon).velocity.m_per_s


	
def vis_visa(radius, semimajor, STANDARD_GRAV):
	return np.sqrt(STANDARD_GRAV * ((2/radius) - (1/semimajor)))
	
def findBurnTime(mass, throttle, massFlux, exhaustC, deltaV):
	burnTime = (mass / massFlux) * (1 - (1/(np.e ** (deltaV/(throttle * exhaustC)))))
	return burnTime