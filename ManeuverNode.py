import datetime
import VectorMath as vecm
import Constants as const

class ManeuverNode:
	def __init__(self, data):
		self.timedelta = data[0]
		self.targetHeading = data[1]
		self.throttle = data[2]
		self.data = data

		self.timedelta = [float(i) for i in self.timedelta.split(":")]
		self.timedelta = datetime.timedelta(self.timedelta[0], self.timedelta[1], self.timedelta[2])
		self.burnTime = datetime.timedelta(days = 0, seconds = 0, microseconds = 0)
		self.drift = False
		self.RCSComplete = False
		self.thrustComplete = False
	
	def interpret_heading(self, rocket):
		if (self.targetHeading == "@"):
			self.targetHeading = vecm.vector(1,0,0)
			self.drift = True
		elif (self.targetHeading == "X"):
			localOrient = rocket.loadLocalOrientation()
			self.targetHeading = rocket.transformToOrientation(
				vecm.vector(1,0,0),  
				const.ECI_X, 
				const.ECI_Y, 
				const.ECI_Z,
				localOrient[0], 
				localOrient[1], 
				localOrient[2])
			self.drift = False
		elif (self.targetHeading == "Y"):
			localOrient = rocket.loadLocalOrientation()
			self.targetHeading = rocket.transformToOrientation(
				vecm.vector(0,1,0),  
				const.ECI_X, 
				const.ECI_Y, 
				const.ECI_Z,
				localOrient[0], 
				localOrient[1], 
				localOrient[2])
			self.drift = False
		elif (self.targetHeading == "Z"):
			localOrient = rocket.loadLocalOrientation()
			self.targetHeading = rocket.transformToOrientation(
				vecm.vector(0,0,1),  
				const.ECI_X, 
				const.ECI_Y, 
				const.ECI_Z,
				localOrient[0], 
				localOrient[1], 
				localOrient[2])
			self.drift = False
		elif (self.targetHeading == "PROGRADE"):
			print("yuh")
			self.targetHeading = vecm.vector(1,0,0)
			self.drift = False
		elif (self.targetHeading == "RETROGRADE"):
			self.targetHeading = vecm.vector(-1,0,0)
			self.drift = False
		elif (self.targetHeading == "RADIALOUT"):
			self.targetHeading = vecm.vector(0,1,0)
			self.drift = False
		elif (self.targetHeading == "RADIALIN"):
			self.targetHeading = vecm.vector(0,-1,0)
			self.drift = False
		elif (self.targetHeading == "NORMAL"):
			self.targetHeading = vecm.vector(0,0,1)
			self.drift = False
		elif (self.targetHeading == "ANTINORMAL"):
			self.targetHeading = vecm.vector(0,0,-1)
			self.drift = False
# 		Manual heading entry
		else:
			headingData = vecm.vector([float(i) for i in self.targetHeading.split("-")])
			self.targetHeading = vecm.unit_vector(vecm.vector(headingData[0],headingData[1],headingData[2]))
# 			Local orientation input without hold heading
			if(headingData[3] == 0):
				self.drift = False
# 			Local orientation input with hold heading
			if(headingData[3] == 1):
				self.targetHeading = rocket.transformToOrientation(
					self.targetHeading, 
					const.ECI_X, 
					const.ECI_Y, 
					const.ECI_Z)
				self.drift = False

				
	
		rocket.targetHeading = self.targetHeading
		rocket.drift = self.drift
			
	def transformToLocalOrientation(vec, localOrient):
		i = vector(1,0,0)
		j = vector(0,1,0)
		k = vector(0,0,1)
		x = localOrient[0]
		y = localOrient[1]
		z = localOrient[2]
		transformedVec = vector(0,0,0)
		transformedVec[0] = (vec[0] * dot(x,i)) + (vec[1] * dot(x,j)) + (vec[2] * dot(x,k))
		transformedVec[1] = (vec[0] * dot(y,i)) + (vec[1] * dot(y,j)) + (vec[2] * dot(y,k))
		transformedVec[2] = (vec[0] * dot(z,i)) + (vec[1] * dot(z,j)) + (vec[2] * dot(z,k))
		return (transformedVec)
		
	def loadLocalOrientation(rocket):
		localX = unit_vector(rocket.pos)
		localY = unit_vector(rocket.vel)
		localZ = unit_vector(rocket.orbitalPlaneNormal())
		return vector(localX, localY, localZ)
			
	def interpret_throttle(self, rocket):
		throttleData = [float(i) for i in self.throttle.split("-")]
		self.throttle = float(throttleData[0]) / 100
		self.burnTime = datetime.timedelta(seconds=float(throttleData[1]))
		
		rocket.throttle = self.throttle
		rocket.burnTime = self.burnTime
		rocket.burnTimeLeft = self.burnTime
		
		