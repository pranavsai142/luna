import VectorMath as vecm
import OrbitalMath as orbitm	
import Constants as const
import datetime

class Rocket:
	def __init__(
		self, 
		init_time, 
		initPos, 
		initVel, 
		initMass, 
		initDryMass, 
		initOrient, 
		specificImpulse, 
		massFlux,
		deltaTheta,
		secondsOfRCS,
		maneuverNodes):
		self.init_time = init_time
		self.pos = initPos
		self.vel = initVel
		self.init_mass = initMass
		self.dryMass = initDryMass
		self.orient = initOrient
		self.specificImpulse = specificImpulse
		self.massFlux = massFlux
		self.deltaTheta = deltaTheta
		self.secondsOfRCS = secondsOfRCS
		# 		All flight nodes get fed into this array
		self.maneuverNodes = maneuverNodes
		
		self.exhaustC = self.specificImpulse * const.EARTH_GRAV_SEA
		self.time = init_time
		localOrient = self.loadLocalOrientation()
		self.heading = self.transformToOrientation(
			self.orient, 
			const.ECI_X, 
			const.ECI_Y, 
			const.ECI_Z, 
			localOrient[0], 
			localOrient[1], 
			localOrient[2])
		self.targetHeading = self.heading
		self.drift = True
		self.mass = initMass
		self.init_fuelMass = self.init_mass - self.dryMass
		self.throttle = 0
		self.secondsOfFuelLeft = datetime.timedelta(seconds = (self.init_fuelMass / self.massFlux))
		self.burnTime =	datetime.timedelta(days=0,seconds=0,microseconds=0)
		self.burnTimeLeft = datetime.timedelta(days=0,seconds=0,microseconds=0)
		self.burnTimeStart = datetime.timedelta(days=0, seconds=0, microseconds=0)
		self.engineOn = False
		self.RCSOn = False
		self.collision = False
		self.secondsOfRCSLeft = self.secondsOfRCS
		
# 		The maneuver index tells which flight node on the flightprofile we are on based on time into flight
		self.maneuverIndex = 0
		
		self.flightdata()
		self.orbitalstateEarth()
		

	def fuelMass(self):
		return self.mass - self.dryMass
		
# 	ENGINE LOGIC
	def outOfFuel(self):
		if(self.secondsOfFuelLeft.total_seconds() <= 0.0):
			return True
		else:
			return False
			
	def endBurn(self):
		if(self.burnTimeLeft.total_seconds() <= 0.0):
			return True
		else:
			return False
		
	def activeBurn(self):
		if(self.burnTimeLeft.total_seconds() > 0.0):
			return True
		else:
			return False
			
	def queryThrust(self, deltaT):
		forceFromRocket = vecm.vector(0,0,0)
		
		if(self.outOfFuel()):
			self.engineOn = False
		elif(self.activeBurn() and not self.engineOn):
				self.engineOn = True

		if(self.engineOn):
			if(self.burnTimeLeft < deltaT):
				deltaT = datetime.timedelta(seconds=(deltaT.total_seconds() * (self.burnTimeLeft/deltaT)))
			if(self.secondsOfFuelLeft < deltaT):
				deltaT = datetime.timedelta(seconds=(self.secondsOfFuelLeft.total_seconds()))
			scalarForceFromRocket = self.throttle * self.exhaustC * self.massFlux * deltaT.total_seconds()
			forceFromRocket = scalarForceFromRocket * self.orient
			self.burnTimeLeft = self.burnTimeLeft - deltaT
			self.mass = self.mass - (self.throttle * self.massFlux * deltaT.total_seconds())
			FullThrottleDeltaT = datetime.timedelta(
				seconds = (deltaT.total_seconds() * self.throttle))
			self.secondsOfFuelLeft = self.secondsOfFuelLeft - FullThrottleDeltaT
		
		if(self.endBurn()):
			self.engineOn = False
			self.throttle = 0.0
			self.burnTime = datetime.timedelta(days=0,seconds=0,microseconds=0)
			self.burnTimeLeft = self.burnTime
			if(self.flightControl()):
				self.maneuverNodes[self.maneuverIndex].RCSComplete = True
			
		return forceFromRocket / self.mass
	
		
# 	RCS Logic
	def outOfRCS(self):
		if(self.secondsOfRCSLeft.total_seconds() <= 0.0):
			return True
		else:
			return False	
			
	def endRCS(self, thetaRotationLeft):
		if(thetaRotationLeft <= 0.0):
			return True
		else:
			return False
			
	def activeRCS(self, thetaRotationLeft):
		if(thetaRotationLeft > 0.0):
			return True
		else:
			return False
	
	def queryRCS(self, deltaT):
		localOrient = self.loadLocalOrientation()
		self.heading = self.transformToOrientation(
			self.orient, 
			const.ECI_X, 
			const.ECI_Y, 
			const.ECI_Z, 
			localOrient[0], 
			localOrient[1], 
			localOrient[2])
		thetaRotationLeft = 0.0
		if(not self.drift):
			if(self.flightControl()):
				if(self.maneuverNodes[self.maneuverIndex].RCSComplete):
					thetaRotationLeft = vecm.angleBetweenVectors(self.heading, self.targetHeading)
		else:
			self.RCSOn = False
		if(self.outOfRCS()):
			self.RCSOn = False
		elif(self.activeRCS(thetaRotationLeft) and not self.RCSOn):
			self.RCSOn = True
		elif(not self.activeRCS(thetaRotationLeft) and self.RCSOn):
			self.RCSOn = False
		
			
			
		if(self.RCSOn):
			deltaRotation = self.deltaTheta
			if(thetaRotationLeft < deltaRotation):
				deltaT = datetime.timedelta(seconds=(deltaT.total_seconds() * (thetaRotationLeft/deltaRotation)))
				deltaRotation = thetaRotationLeft
			if(self.secondsOfRCSLeft < deltaT):
				fractionalDeltaT = datetime.timedelta(seconds=(secondsOfRCSLeft.total_seconds()))
				deltaRotation = deltaRotation * (fractionalDeltaT.total_seconds()/deltaT.total_seconds())
				deltaT = fractionalDeltaT
			rotationAxis = vecm.cross_vector(self.heading, self.targetHeading)
			self.heading = vecm.rotate_around_arbitrary(self.heading, rotationAxis, deltaRotation)
			thetaRotationLeft = thetaRotationLeft - deltaRotation
			self.secondsOfRCSLeft = self.secondsOfRCSLeft - deltaT
			self.orient = self.transformToOrientation(
				self.heading, 
				localOrient[0], 
				localOrient[1], 
				localOrient[2],
				const.ECI_X,
				const.ECI_Y,
				const.ECI_Z)
				
		if(self.endRCS(thetaRotationLeft)):
			self.RCSOn = False
			if(self.flightControl()):
				self.maneuverNodes[self.maneuverIndex].RCSComplete = True
			
			
			
	def orbitalPlaneNormal(self):
		return vecm.cross_vector(self.pos, self.vel)
	
	def loadFlightProfile(self, maneuverNodes):
		if(len(maneuverNodes) > 0):
			self.maneuverNodes = maneuverNodes
			self.maneuverIndex = 0
	
	def flightdata(self):
		return {
			"time": self.time,
			"pos": self.pos,
			"vel": self.vel,
			"mass": self.mass,
			"fuelMass": self.fuelMass(),
			"secondsOfFuelLeft": self.secondsOfFuelLeft.total_seconds(),
			"secondsOfRCSLeft": self.secondsOfRCSLeft.total_seconds(),
			"orient": self.orient,
			"heading": self.heading,
			"targetHeading": self.targetHeading,
			"collision": self.collision,
			"drift": self.drift,
			"engineOn": self.engineOn,
			"throttle": self.throttle,
			"currentManeuverNode": self.currentManeuverNode()
		}
		
			
	def currentManeuverNode(self):
		node = 0
		lenManeuverNodes = len(self.maneuverNodes)
		if(lenManeuverNodes == 0 or self.maneuverIndex >= lenManeuverNodes):
			node = None
		else:
			node = self.maneuverNodes[self.maneuverIndex]
		return node
		
	def flightControl(self):
		lenManeuverNodes = len(self.maneuverNodes)
		if(lenManeuverNodes == 0 or self.maneuverIndex >= lenManeuverNodes):
			return False
		else:
			return True
			
	def beginManeuver(self):
		if(self.maneuverIndex < len(self.maneuverNodes)):
			if(self.maneuverNodes[self.maneuverIndex].timedelta <= self.timedelta()):
				return True
		else:
			return False
			
	def timedelta(self):
		return self.time - self.init_time
		
	def advanceTime(self, deltaT):
		self.queryRCS(deltaT)
		impulseFromRocket = self.queryThrust(deltaT)
		self.time = self.time + deltaT
		if(self.beginManeuver()):
			self.maneuverNodes[self.maneuverIndex].interpret_heading(self)
			self.maneuverNodes[self.maneuverIndex].interpret_throttle(self)
			self.maneuverIndex = self.maneuverIndex + 1
		return impulseFromRocket
	
	def rungekuttadvanceTime(self, duration):
		delta_t = datetime.timedelta(seconds = (duration.total_seconds() / 3))
		accelerationsFromRocket = []
		for i in range(3):
			self.queryRCS(delta_t)
			accelerationsFromRocket.append(self.queryThrust(delta_t))
			self.time = self.time + delta_t
			if(self.beginManeuver()):
				self.maneuverNodes[self.maneuverIndex].interpret_heading(self)
				self.maneuverNodes[self.maneuverIndex].interpret_throttle(self)
				self.maneuverIndex = self.maneuverIndex + 1
		return accelerationsFromRocket


# 	Transform a vector vec from one coordinate system <i, j, k> to another <x, y, z>
	def transformToOrientation(self, vec, i, j, k, x, y, z):
		transformedVec = vecm.vector(0.0,0.0,0.0)
		transformedVec[0] = ((vec[0] * vecm.dot(x,i)) + (vec[1] * vecm.dot(x,j)) + (vec[2] * vecm.dot(x,k)))
		transformedVec[1] = ((vec[0] * vecm.dot(y,i)) + (vec[1] * vecm.dot(y,j)) + (vec[2] * vecm.dot(y,k)))
		transformedVec[2] = ((vec[0] * vecm.dot(z,i)) + (vec[1] * vecm.dot(z,j)) + (vec[2] * vecm.dot(z,k)))
		return (vecm.unit_vector(transformedVec))
		
	def loadLocalOrientation(self):
		localX = vecm.unit_vector(self.vel)
		localY = vecm.unit_vector(self.pos)
		localZ = vecm.unit_vector(self.orbitalPlaneNormal())
		return (localX, localY, localZ)
		
	def orbitalstateEarth(self):
		craftLatLongHeight = orbitm.to_lat_long_height_from_eci(
			self.pos[0], 
			self.pos[1], 
			self.pos[2], 
			self.time, 
			const.EARTH_FLATTEN_CONST, 
			const.EARTH_A_RADIUS)
		craftLat = craftLatLongHeight[0]
		craftLong = craftLatLongHeight[1]
		craftHeight = craftLatLongHeight[2]
		radiusToEarthCenter = vecm.mag(self.pos)
		velocityMag = vecm.mag(self.vel)
		semimajor = orbitm.find_semimajorly(radiusToEarthCenter, velocityMag, const.EARTH_STANDARD_GRAV)
		eccentricityEarthVector = orbitm.find_eccentricity_vector(self.pos, self.vel, const.EARTH_STANDARD_GRAV)
		eccentricityEarth = vecm.mag(eccentricityEarthVector)
		apoPeri = orbitm.find_apo_peri(eccentricityEarth, semimajor)
		apoapsis = apoPeri[0]
		periapsis = apoPeri[1]
		flightAngle = orbitm.find_flight_angle_state_vectors(self.pos, self.vel)
		inclination = orbitm.extrapolate_inclination(self.pos, self.vel)
		trueanomaly = orbitm.find_true_anomaly(self.pos, self.vel, eccentricityEarthVector)
		timeAnomaliesEarth = orbitm.find_time_anomalies(semimajor, trueanomaly, const.EARTH_STANDARD_GRAV)
		orbitalPeriodEarth = timeAnomaliesEarth[0]
		timeSincePeriapsisEarth = timeAnomaliesEarth[1]
		timeToPeriapsisEarth = timeAnomaliesEarth[2]
		return {
			"craftLat": craftLat,
			"craftLong": craftLong,
			"craftHeight": craftHeight,
			"radiusToEarthCenter": radiusToEarthCenter,
			"semimajor": semimajor,
			"eccentricity": eccentricityEarth,
			"apoapsis": apoapsis,
			"periapsis": periapsis,
			"flightAngle": flightAngle,
			"inclination": inclination,
			"trueanomaly": trueanomaly,
			"orbitalPeriod": orbitalPeriodEarth,
			"timeSincePeriapsis": timeSincePeriapsisEarth,
			"timeToPeriapsis": timeToPeriapsisEarth
		}
		
	def orbitalstateMoon(self, moonLocation, moonVel):
		posMoon = moonLocation - self.pos
		velMoon = moonVel + self.vel
# 		Calculate selenographic latitude and longitude of craft. 
		craftLatLongSelenographic = orbitm.find_moon_lat_long(posMoon)
		craftLatSelenographic = craftLatLongSelenographic[0]
		craftLongSelenographic = craftLatLongSelenographic[1]
		eccentricityMoon = orbitm.find_eccentricity(posMoon, velMoon, const.MOON_STANDARD_GRAV)
# 		Find radius of moon at discovered selenographic latitude
		radiusToMoonSurface = orbitm.find_radius_from_body(
			craftLatSelenographic, 
			const.MOON_A_RADIUS, 
			const.MOON_FLATTEN_CONST, 
			0)
		radiusToMoonCenter = vecm.mag(posMoon)
# 	Get unit vector pointing in direction of craft from center of moon
		unitPosMoon = vecm.unit_vector(posMoon)
# 		Create vector of magnitude moon's radius pointing from the center of the moon to craft
		moonRadiusVector = radiusToMoonSurface * unitPosMoon
# 		Subtract center of moon to craft minus center of moon to surface to get surface to craft
		surfaceOfMoonToCraft = posMoon - moonRadiusVector
# 		Calculate magnitude of vector to give meters above surface
# 		This distance is a rough estimate of the craft's altitude above moon based
# 		On flatten ellipsoid model of moon
		distanceFromMoonSurface = vecm.mag(surfaceOfMoonToCraft)
		velocityMagMoon = vecm.mag(velMoon)
		semimajor = orbitm.find_semimajorly(radiusToMoonCenter, velocityMagMoon, const.MOON_STANDARD_GRAV)
		eccentricityMoonVector = orbitm.find_eccentricity_vector(posMoon, velMoon, const.EARTH_STANDARD_GRAV)
		eccentricityMoon = vecm.mag(eccentricityMoonVector)
		trueanomaly = orbitm.find_true_anomaly(self.pos, self.vel, eccentricityMoonVector)
		timeAnomaliesMoon = orbitm.find_time_anomalies(semimajor, trueanomaly, const.MOON_STANDARD_GRAV)
		orbitalPeriodMoon = timeAnomaliesMoon[0]
		timeSincePeriapsisMoon = timeAnomaliesMoon[1]
		timeToPeriapsisMoon = timeAnomaliesMoon[2]
		
		

# 		craftLatLongHeight = to_lat_long_height_from_eci(
# 			pos[0], 
# 			pos[1], 
# 			pos[2], 
# 			time, 
# 			EARTH_FLATTEN_CONST, 
# 			EARTH_A_RADIUS)
# 		craftLat = craftLatLongHeight[0]
# 		craftLong = craftLatLongHeight[1]
# 		craftHeight = craftLatLongHeight[2]
# 		radiusToEarthCenter = find_radius_from_body(craftLat, EARTH_A_RADIUS, EARTH_FLATTEN_CONST, craftHeight)
# 		velocityMag = mag(vel)
# 		semimajor = find_semimajorly(radiusToEarthCenter, velocityMag, EARTH_STANDARD_GRAV)
# 		eccentricityEarthVector = find_eccentricity_vector(pos, vel, EARTH_STANDARD_GRAV)
# 		eccentricityEarth = mag(eccentricityEarthVector)
# 		apoPeri = find_apo_peri(eccentricityEarth, semimajor)
# 		flightAngle = find_flight_angle_state_vectors(pos, vel)
# 		inclination = m.degrees(extrapolate_inclination(pos, vel))
# 		trueanomaly = find_true_anomaly(pos, vel, eccentricityEarthVector)
		return {
			"posMoon": posMoon,
			"eccentricity": eccentricityMoon,
			"distanceFromMoonSurface": distanceFromMoonSurface,
			"radiusToMoonCenter": radiusToMoonCenter,
			"orbitalPeriod": orbitalPeriodMoon,
			"timeSincePeriapsis": timeSincePeriapsisMoon,
			"timeToPeriapsisMoon": timeToPeriapsisMoon
		}
		