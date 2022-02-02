import numpy as np
import datetime
import sys
import copy
from skyfield.api import load
import matplotlib.pyplot as plt

import Constants as const
import VectorMath as vecm
import OrbitalMath as orbitm

import Rocket
import ManeuverNode
import ConditionNode

	
class Grapher:
	craftHeight = []
	posX = []
	posY = []
	posZ = []
	radiusToEarthCenter = []
	craftLat = []
	craftLong = []
	apos = []
	peris = []
	flightAngles = []
	inclinations = []
	eccentricitysEarth = []
	eccentricitysMoon = []
	distanceFromMoonSurface = []
	throttle = []
	engineOn = []
	velMag = []
	fuelMass = []
	secondsOfFuelLeft = []
	secondsOfRCSLeft = []
	orbitalPeriod = []
	timeSincePeriapsis = []
	timeToPeriapsis = []
	moonLocationX = []
	moonLocationY = []



def takeOff(rocket, init_time, apogee, perigee, inclination):
	print("derbbbbb")

def startSimulation(rocket):
	# Init position and velocity in LEO to start the simulation at. 
	continueSimulation = True
	rocket.collision = False
	earthPos = vecm.vector(0, 0, 0)
	grapher = Grapher()
	time = rocket.time

	while(continueSimulation):				
	
		flightdata = rocket.flightdata()
# 		J2000 coordinates of moon at tick time
		moonLocation = orbitm.getMoonLocation(
			flightdata["time"].year, 
			flightdata["time"].month, 
			flightdata["time"].day, 
			flightdata["time"].hour, 
			flightdata["time"].minute, 
			flightdata["time"].second,
			flightdata["time"].microsecond)
		moonVelocity = orbitm.getMoonVelocity(
			flightdata["time"].year, 
			flightdata["time"].month, 
			flightdata["time"].day, 
			flightdata["time"].hour, 
			flightdata["time"].minute, 
			flightdata["time"].second,
			flightdata["time"].microsecond)
 		
		
# 		Receive rocket data

		osEarth = rocket.orbitalstateEarth()
		osMoon = rocket.orbitalstateMoon(moonLocation, moonVelocity)
		
# 		print(flightdata)
# 		print(osEarth)
# 		print(osMoon)

# 		Collision detection
		if(osEarth["craftHeight"] <= 0.0):
			print("COLLISION WITH EARTH SURFACE")
			collision = True

# 		Calculate magnitude of force of gravity on craft
		scalarForceFromEarth = (const.G * rocket.mass * const.EARTH_MASS) / (osEarth["radiusToEarthCenter"] ** 2)
# 		Get vector pointing TO (-1) center of earth.
		unitCraftToEarthPosition = vecm.unit_vector(rocket.pos) * -1
# 		Get force vector of earths gravity
		forceFromEarth = scalarForceFromEarth * unitCraftToEarthPosition
	
		grapher.craftHeight.append(osEarth["craftHeight"])
		grapher.radiusToEarthCenter.append(osEarth["radiusToEarthCenter"])
		grapher.posX.append(flightdata["pos"][0])
		grapher.posY.append(flightdata["pos"][1])
		grapher.posZ.append(flightdata["pos"][2])
		
		grapher.moonLocationX.append(moonLocation[0])
		grapher.moonLocationY.append(moonLocation[1])
		
		
		grapher.craftLat.append(osEarth["craftLat"])
		grapher.craftLong.append(osEarth["craftLong"])
		
# 		print("eccentricityEarth", osEarth["eccentricity"])
		grapher.eccentricitysEarth.append(osEarth["eccentricity"])
		
# 		print("eccentricityMoon", osMoon["eccentricity"])
		grapher.eccentricitysMoon.append(osMoon["eccentricity"])
		
# 		print("radiusToEarthCenter", osEarth["radiusToEarthCenter"])
# 		print("semimajor(ly)", osEarth["semimajor"])
# 		print("apoPeri", osEarth["apoPeri"])
		grapher.apos.append(osEarth["apoapsis"])
		grapher.peris.append(osEarth["periapsis"])
		
		grapher.flightAngles.append(osEarth["flightAngle"])
# 		print("flightAngle", np.degrees(osEarth["flightAngle"]))

# 		print("inclination", osEarth["inclination"])
		grapher.inclinations.append(osEarth["inclination"])
		
# 		print("distance from moon", osMoon["distanceFromMoonSurface"])
		grapher.distanceFromMoonSurface.append(osMoon["distanceFromMoonSurface"])
		

		grapher.throttle.append(flightdata["throttle"])
		grapher.velMag.append(vecm.mag(flightdata["vel"]))
		if(flightdata["engineOn"]):
			grapher.engineOn.append(1)
		else:
			grapher.engineOn.append(0)
			
		grapher.fuelMass.append(flightdata["fuelMass"])
		
# 		print("heading", flightdata["heading"])
# 		print("orient", flightdata["orient"])
# 		print("targetHeading", flightdata["targetHeading"])
		
		grapher.secondsOfRCSLeft.append(flightdata["secondsOfRCSLeft"])
		grapher.secondsOfFuelLeft.append(flightdata["secondsOfFuelLeft"])
		
		if(type(osEarth["timeSincePeriapsis"]) is datetime.timedelta):
			grapher.timeSincePeriapsis.append(osEarth["timeSincePeriapsis"].total_seconds())
			grapher.orbitalPeriod.append(osEarth["orbitalPeriod"])
			grapher.timeToPeriapsis.append(osEarth["timeToPeriapsis"].total_seconds())
		else:
			grapher.timeSincePeriapsis.append(-1)
			grapher.orbitalPeriod.append(-1)
			grapher.timeToPeriapsis.append(-1)
		

# 		Calculate scalar force of gravity from moon
		scalarForceFromMoon = (const.G * rocket.mass * const.MOON_MASS) / (osMoon["radiusToMoonCenter"]**2)
# 		Negate unit vector from center of moon to craft to give craft to center of moon
		unitCraftToMoonPosition = vecm.unit_vector(osMoon["posMoon"]) * -1
# 		Force vector from force of gravity of moon
		forceFromMoon = scalarForceFromMoon * unitCraftToMoonPosition
		
		
# 		Conservation of Momentum over delta_t
# 		init momentum + sum of forces * delta_t = final_momentum
# 		Calculates new position and velocity based on forces

		if(not rocket.collision):
# 			initMomentum = rocket.mass * rocket.vel
# 			sumOfImpulse = forceFromEarth * const.delta_t.total_seconds()
# 			sumOfImpulse = sumOfImpulse + (forceFromMoon * const.delta_t.total_seconds())
# 			sumOfImpulse = sumOfImpulse + (rocket.advanceTime(const.delta_t))
# 
# 			finalMomentum = initMomentum + sumOfImpulse
# 			finalVel = finalMomentum / rocket.mass
# 			finalPos = rocket.pos + (finalVel * const.delta_t.total_seconds())
# 			
# 			rocket.pos = finalPos
# 			rocket.vel = finalVel
			rocketAcc = rocket.rungekuttadvanceTime(const.hstep)
			k1Acc = findAccelerationFromEarth(flightdata["pos"]) + findAccelerationFromMoon(osMoon["posMoon"]) + rocketAcc[0]
			k1Vel = flightdata["vel"]
			k2Pos = flightdata["pos"] + (k1Vel * const.hstep_total_seconds/2)
			moonk2k3Location = orbitm.getMoonLocation(
				flightdata["time"].year, 
				flightdata["time"].month, 
				flightdata["time"].day, 
				flightdata["time"].hour, 
				flightdata["time"].minute, 
				(flightdata["time"].second + const.hstep_total_seconds/2),
				flightdata["time"].microsecond)
			k2k3PosMoon = moonLocation - flightdata["pos"]
			k2Acc = findAccelerationFromEarth(k2Pos) + findAccelerationFromMoon(k2k3PosMoon) + rocketAcc[1]
			k2Vel = flightdata["vel"] + (k1Acc * const.hstep_total_seconds/2)
			k3Pos = flightdata["pos"] + (k2Vel * const.hstep_total_seconds/2)
			k3Acc = findAccelerationFromEarth(k3Pos) + findAccelerationFromMoon(k2k3PosMoon) + rocketAcc[1]
			k3Vel = flightdata["vel"] + (k2Acc * const.hstep_total_seconds/2)
			k4Pos = flightdata["pos"] + (k3Vel * const.hstep_total_seconds)
			moonk4Location = orbitm.getMoonLocation(
				flightdata["time"].year, 
				flightdata["time"].month, 
				flightdata["time"].day, 
				flightdata["time"].hour, 
				flightdata["time"].minute, 
				(flightdata["time"].second + const.hstep_total_seconds),
				flightdata["time"].microsecond)
			k4PosMoon = moonLocation - flightdata["pos"]
			k4Acc = findAccelerationFromEarth(k4Pos) + findAccelerationFromMoon(k4PosMoon) + rocketAcc[2]
			k4Vel = flightdata["vel"] + (k3Acc * const.hstep_total_seconds)
			finalVel = flightdata["vel"] + (const.hstep_total_seconds/6) * (k1Acc + (2 * k2Acc) + (2 * k3Acc) + k4Acc)
			finalPos = flightdata["pos"] + (const.hstep_total_seconds/6) * (k1Vel + (2 * k2Vel) + (2 * k3Vel) + k4Vel)
			
			rocket.pos = finalPos
			rocket.vel = finalVel
			

		
		
		if((rocket.time - rocket.init_time).total_seconds() >= const.simulationduration):
			print("hour passed")
			plt.figure(1)
			plt.plot(grapher.craftHeight, ".r", label="height from earth surface")
			plt.legend()
			plt.figure(2)
			plt.plot(grapher.craftLong, grapher.craftLat, ".r", label="craft lat long")
			plt.legend()
			plt.figure(3)
			plt.plot(grapher.inclinations, ".r", label="inclination")
			plt.legend()
			plt.figure(4)
			plt.plot(grapher.posX, grapher.posY, ".r", label="Craft Location ECI XY parametric")
			plt.legend()
			plt.figure(5)
			plt.plot(grapher.moonLocationX, grapher.moonLocationY, ".r", label="Moon Location ECI XY parametric")
			plt.legend()
# 			plt.figure(6)
# 			plt.plot(grapher.distanceFromMoonSurface, ".r", label="distance from moon surface")
# 			plt.legend()
			plt.figure(7)
			plt.plot(grapher.timeSincePeriapsis, ".r", label="timeSincePeriapsis")
			plt.plot(grapher.timeToPeriapsis, ".g", label="timeToPeriapsis")
			plt.plot(grapher.orbitalPeriod, ".b", label="orbitalPeriod")
			plt.legend()
			plt.figure(8)
			plt.plot(grapher.apos, ".g", label="apoapsis")
			plt.plot(grapher.peris, ".b", label="periapsis")
			plt.plot(grapher.radiusToEarthCenter, ".r", label="radiusToEarthCenter")
			plt.legend()
			plt.figure(9)
			plt.plot(grapher.posX, ".r", label="x")
			plt.plot(grapher.posY, ".g", label="y")
			plt.plot(grapher.posZ, ".b", label="z")
			plt.legend()
			plt.show()
			
# 			plt.figure(1)
# 			plt.plot(grapher.engineOn, "or", label="engineOn")
# 			plt.legend()
# 			plt.figure(2)
# 			plt.plot(grapher.throttle, "or", label="throttle")
# 			plt.legend()
# 			plt.figure(3)
# 			plt.plot(grapher.velMag, "or", label="velMag")
# 			plt.legend()
# 			plt.figure(4)
# 			plt.plot(grapher.fuelMass, "or", label="fuelMass")
# 			plt.legend()
# 			plt.figure(5)
# 			plt.plot(grapher.secondsOfRCSLeft, "or", label="secondsOfRCSLeft")
# 			plt.legend()
# 			plt.figure(6)
# 			plt.plot(grapher.secondsOfFuelLeft, "or", label="secondsOfFuelLeft")

			plt.show()
			userContinueSimulation = input("continue? (y or n): ")
			if(userContinueSimulation == "n"):
				continueSimulation = False
		
		if(rocket.collision == True):
			userContinueSimulation = input("continue? (y or n): ")
			if(userContinueSimulation == "n"):
				continueSimulation = False
		
def warpToPeriapsis(rocket):
	osEarth = rocket.orbitalstateEarth()
	warpshift = osEarth["timeToPeriapsis"]
	simRocket = warpTimeAndReturnNewTimeWarpedRocket(rocket, warpshift, const.delta_t)
	osSimEarth = simRocket.orbitalstateEarth()
	secThreshold = datetime.timedelta(seconds = 1)
	while(osSimEarth["timeSincePeriapsis"] > secThreshold and osEarth["timeToPeriapsis"] >= warpshift and warpshift > osSimEarth["timeSincePeriapsis"]):
		warpshift = warpshift - osSimEarth["timeSincePeriapsis"]
		simRocket = warpTimeAndReturnNewTimeWarpedRocket(rocket, warpshift, const.delta_t)
		osSimEarth = simRocket.orbitalstateEarth()
	print("passed rocket", osEarth)
	print()
	print("warped rocket to perigee", osSimEarth)
	return simRocket

def warpTimeAndReturnNewTimeWarpedRocket(originalRocket, duration, delta_t):
	rocket = copy.deepcopy(originalRocket)
		# Init position and velocity in LEO to start the simulation at. 
	continueSimulation = True
	rocket.collision = False
	earthPos = vecm.vector(0, 0, 0)
	start_time = rocket.time

	while(continueSimulation):	
	# 		Receive rocket data			
		flightdata = rocket.flightdata()
# 		J2000 coordinates of moon at tick time
		moonLocation = orbitm.getMoonLocation(
			flightdata["time"].year, 
			flightdata["time"].month, 
			flightdata["time"].day, 
			flightdata["time"].hour, 
			flightdata["time"].minute, 
			flightdata["time"].second,
			flightdata["time"].microsecond)
		moonVelocity = orbitm.getMoonVelocity(
			flightdata["time"].year, 
			flightdata["time"].month, 
			flightdata["time"].day, 
			flightdata["time"].hour, 
			flightdata["time"].minute, 
			flightdata["time"].second,
			flightdata["time"].microsecond)
 		
		
# 		Receive rocket data

		osEarth = rocket.orbitalstateEarth()
		osMoon = rocket.orbitalstateMoon(moonLocation, moonVelocity)
		
# 		print(flightdata)
# 		print(osEarth)
# 		print(osMoon)

# 		Collision detection
		if(osEarth["craftHeight"] <= 0.0):
			print("COLLISION WITH EARTH SURFACE")
			rocket.collision = True

		

# 		Calculate scalar force of gravity from moon
		scalarForceFromMoon = (const.G * rocket.mass * const.MOON_MASS) / (osMoon["radiusToMoonCenter"]**2)
# 		Negate unit vector from center of moon to craft to give craft to center of moon
		unitCraftToMoonPosition = vecm.unit_vector(osMoon["posMoon"]) * -1
# 		Force vector from force of gravity of moon
		forceFromMoon = scalarForceFromMoon * unitCraftToMoonPosition
		
		
# 		Conservation of Momentum over delta_t
# 		init momentum + sum of forces * delta_t = final_momentum
# 		Calculates new position and velocity based on forces

		if(not rocket.collision):
			rocketAcc = rocket.rungekuttadvanceTime(const.hstep)
			k1Acc = findAccelerationFromEarth(flightdata["pos"]) + findAccelerationFromMoon(osMoon["posMoon"]) + rocketAcc[0]
			k1Vel = flightdata["vel"]
			k2Pos = flightdata["pos"] + (k1Vel * const.hstep_total_seconds/2)
			moonk2k3Location = orbitm.getMoonLocation(
				flightdata["time"].year, 
				flightdata["time"].month, 
				flightdata["time"].day, 
				flightdata["time"].hour, 
				flightdata["time"].minute, 
				(flightdata["time"].second + const.hstep_total_seconds/2),
				flightdata["time"].microsecond)
			k2k3PosMoon = moonLocation - flightdata["pos"]
			k2Acc = findAccelerationFromEarth(k2Pos) + findAccelerationFromMoon(k2k3PosMoon) + rocketAcc[1]
			k2Vel = flightdata["vel"] + (k1Acc * const.hstep_total_seconds/2)
			k3Pos = flightdata["pos"] + (k2Vel * const.hstep_total_seconds/2)
			k3Acc = findAccelerationFromEarth(k3Pos) + findAccelerationFromMoon(k2k3PosMoon) + rocketAcc[1]
			k3Vel = flightdata["vel"] + (k2Acc * const.hstep_total_seconds/2)
			k4Pos = flightdata["pos"] + (k3Vel * const.hstep_total_seconds)
			moonk4Location = orbitm.getMoonLocation(
				flightdata["time"].year, 
				flightdata["time"].month, 
				flightdata["time"].day, 
				flightdata["time"].hour, 
				flightdata["time"].minute, 
				(flightdata["time"].second + const.hstep_total_seconds),
				flightdata["time"].microsecond)
			k4PosMoon = moonLocation - flightdata["pos"]
			k4Acc = findAccelerationFromEarth(k4Pos) + findAccelerationFromMoon(k4PosMoon) + rocketAcc[2]
			k4Vel = flightdata["vel"] + (k3Acc * const.hstep_total_seconds)
			finalVel = flightdata["vel"] + (const.hstep_total_seconds/6) * (k1Acc + (2 * k2Acc) + (2 * k3Acc) + k4Acc)
			finalPos = flightdata["pos"] + (const.hstep_total_seconds/6) * (k1Vel + (2 * k2Vel) + (2 * k3Vel) + k4Vel)
			
			rocket.pos = finalPos
			rocket.vel = finalVel
			

		if((rocket.time - start_time) >= duration):
			print("warp over")
			continueSimulation = False
			return rocket
			
# def rungekutta(rocket, duration, delta_t):
# 	
# 	continueSimulation = True
# 	rocket.collision = False
# 	earthPos = vecm.vector(0, 0, 0)
# 	start_time = rocket.time
# 
# 	while(continueSimulation):		
# # 		J2000 coordinates of moon at tick time
# 		moonLocation = earth.at(ts.utc(
# 			rocket.time.year, 
# 			rocket.time.month, 
# 			rocket.time.day, 
# 			rocket.time.hour, 
# 			rocket.time.minute, 
# 			(rocket.time.second + (rocket.time.microsecond*1e-6)))).observe(moon).position.m
# 		moonVel = earth.at(ts.utc(
# 			rocket.time.year, 
# 			rocket.time.month, 
# 			rocket.time.day, 
# 			rocket.time.hour, 
# 			rocket.time.minute, 
# 			(rocket.time.second + (rocket.time.microsecond*1e-6)))).observe(moon).velocity.m_per_s
#  		
# 		
# # 		Receive rocket data
# 		flightdata = rocket.flightdata()
# 		osEarth = rocket.orbitalstateEarth()
# 		osMoon = rocket.orbitalstateMoon(moonLocation, moonVel)
# 		
# # 		print(flightdata)
# # 		print(osEarth)
# # 		print(osMoon)
# 
# # 		Collision detection
# 		if(osEarth["craftHeight"] <= 0.0):
# 			print("COLLISION WITH EARTH SURFACE")
# 			rocket.collision = True
# 		
# 		if(not rocket.collision):
# 			k1Acc = findAccelerationFromEarth(flightdata["pos"], flightdata["mass"])
# 			k1Vel = flightdata["vel"]
# 			k2Pos = flightdata["pos"] + (k1Vel * const.hstep_total_seconds/2)
# 			k2Acc = findAccelerationFromEarth(k2Pos, flightdata["mass"])
# 			k2Vel = (flightdata["vel"] + (k1Acc * const.hstep_total_seconds/2)
# 			k3Pos = flightdata["pos"] + (k2Vel * const.hstep_total_seconds/2)
# 			k3Acc = findAccelerationFromEarth(k3Pos, flightdata["mass"])
# 			k3Vel = (flightdata["vel"] + (k2Acc * const.hstep_total_seconds/2)
# 			k4Pos = flightdata["pos"] + (k3Vel * const.hstep_total_seconds)
# 			k4Acc = findAccelerationFromEarth(k4Pos, flightdata["mass"])
# 			k4Vel = (flightdata["vel"] + k3Acc * const.hstep_total_seconds)
# 			finalVel = flightdata["vel"] + (const.hstep_total_seconds/6) * (k1Acc + (2 * k2Acc) + (2 * k3Acc) + k4Acc)
# 			finalPos = flightdata["pos"] + (const.hstep_total_seconds/6) * (k1Vel + (2 * k2Vel) + (2 * k3Vel) + k4Vel)
# 			rocket.advanceTime(const.hstep)
# 			rocket.pos = finalPos
# 			rocket.vel = finalVel
# 			


def findAccelerationFromEarth(pos):
	#Calculate magnitude of force of gravity on craft
	scalarAccelerationFromEarth = (const.G * const.EARTH_MASS) / (vecm.mag(pos) ** 2)
# 		Get vector pointing TO (-1) center of earth.
	unitCraftToEarthPosition = vecm.unit_vector(pos) * -1
# 		Get force vector of earths gravity
	accelerationFromEarth = scalarAccelerationFromEarth * unitCraftToEarthPosition
	return accelerationFromEarth
		
def findAccelerationFromMoon(posMoon):
# 	Calculate scalar force of gravity from moon
	scalarAccelerationFromMoon = (const.G * const.MOON_MASS) / (vecm.mag(posMoon)**2)
# 	Negate unit vector from center of moon to craft to give craft to center of moon
	unitCraftToMoonPosition = vecm.unit_vector(posMoon) * -1
# 	Force vector from force of gravity of moon
	accelerationFromMoon = scalarAccelerationFromMoon * unitCraftToMoonPosition
	return accelerationFromMoon
