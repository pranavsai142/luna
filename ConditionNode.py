import datetime
import numpy as np

import VectorMath as vecm
import OrbitalMath as orbitm
import Constants as const

import Simulation

class ConditionNode:
	def __init__(self, conditionStr):
		self.conditionStr = conditionStr

		conditionArr = self.conditionStr.split(":")
		self.tag = conditionArr[0]
		self.data = [float(i) for i in conditionArr[1].split(",")]
		print("tag", self.tag)
		print("data", self.data)
	
	def createManeuverList(self, rocket):
		if(self.tag == "TAKEOFFEARTH"):
			print("WEEEPERs")
		if(self.tag == "LANDEARTH"):
			print("WEEEPERs")
		if(self.tag == "HOPEARTH"):
			print("WEEEPERs")
		if(self.tag == "TRANSFEREARTH"):
# 			Target apo and peri to hohmann transfer to. Adding earth's average radius to computational metrics
			TARGET_APOAPSIS = self.data[0] + const.EARTH_AVG_RADIUS
			TARGET_PERIAPSIS = self.data[1] + const.EARTH_AVG_RADIUS
			TARGET_INCLINATION = np.radians(self.data[2])
			print("TARGET APOAPSIS, PERIAPSIS (METERS)", TARGET_APOAPSIS, TARGET_PERIAPSIS)
			
# 			Generate os
			osEarth = rocket.orbitalstateEarth()
# 			generate a new rocket that is warped to the periapsis of the orbit
			periRocket = Simulation.warpToPeriapsis(rocket)
			
# 			generate data and orbital velocity vectors for the rocket at periapsis
			periflightdata = periRocket.flightdata()
			periosEarth = periRocket.orbitalstateEarth()
			perivelMag = vecm.mag(periflightdata["vel"])
			periunitVel = vecm.unit_vector(periflightdata["vel"])
			periVisVisa = orbitm.vis_visa(periosEarth["radiusToEarthCenter"], periosEarth["semimajor"], const.EARTH_STANDARD_GRAV)
			
# 			Generate orbital velocity vectors for transfer orbiti
			peritransferSemimajor = orbitm.find_semimajor(TARGET_APOAPSIS, TARGET_PERIAPSIS)
			peritransferVisVisa = orbitm.vis_visa(periosEarth["radiusToEarthCenter"], peritransferSemimajor, const.EARTH_STANDARD_GRAV)
# 			In next iteration incorporate inclination to the deltaV vector to this line
			peritransferVel = peritransferVisVisa * periunitVel
			peritransferunitVel = vecm.unit_vector(peritransferVel)
			
# 			deltaV maneuver vector
			peritransferdeltaVel = peritransferVel - periflightdata["vel"]
			
# 			deltaV (meters/second required)
			peritransferdeltaV = vecm.mag(peritransferdeltaVel)
			
# 			Compute needed time for engine to run to complete burn.
			peritransferBurnTime = orbitm.findBurnTime(periflightdata["mass"], periflightdata["throttle"], periRocket.massFlux, periRocket.exhaustC, peritransferdeltaV)
			print("BURN (SECONDS):", peritransferBurnTime)
			
# 			Create timestamps of burn start and end times, maneuvering start and end times
			peritransferBurnStartTime = periflightdata["time"] - (datetime.timedelta(seconds = (peritransferBurnTime/2)))
			peritransferBurnEndTime = periflightdata["time"] + (datetime.timedelta(seconds = (peritransferBurnTime/2)))
			peritransferOrientation = vecm.unit_vector(peritransferdeltaVel)
			peritransferManeuverStartTime = peritransferBurnStartTime - const.RCS_TIME_BUFFER
			peritransferManeuverEndTime = peritransferBurnEndTime + const.RCS_TIME_BUFFER
			
# 			Calculate timedeltas from t0 to feed the data into a maneuver node# 
# 			periTransferBurnStartT = peritransferBurnStartTime - 
		if(self.tag == "TAKEOFFMOON"):
			print("WEEEPERs")
		if(self.tag == "LANDMOON"):
			print("WEEEPERs")
		if(self.tag == "HOPMOON"):
			print("WEEEPERs")
		if(self.tag == "TRANSFERMOON"):
			print("WEEEPERs")
		
