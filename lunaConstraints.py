import numpy as np
import matplotlib.pyplot as plt
import Constants as const




def findDeltaV(Isp_vac, init_mass, dry_mass):
	c_vac = Isp_vac * const.EARTH_GRAV_AVG
	deltaV = 0
	if(dry_mass > 0 and init_mass >= dry_mass):
		mass_ratio = init_mass/dry_mass
		deltaV = c_vac * np.log(mass_ratio)
	return deltaV
	
def findDeltaVRequiredForHop(height, distance, GRAVITY):
	verticalSpeed = 1
	horizontalSpeed = 1
	verticalAcceleration = GRAVITY + verticalSpeed
	hoverAcceleration = GRAVITY
	verticalDecelleration = GRAVITY - verticalSpeed
	deltaV = verticalAcceleration * (height / verticalSpeed)
	deltaV = deltaV + (hoverAcceleration * (distance / horizontalSpeed)) + (horizontalSpeed * 2)
	deltaV = deltaV + (verticalDecelleration * (height / verticalSpeed))
	return deltaV	
		
def findTankVolume(radius, height):
	volume = 4 * np.pi * (radius ** 2)
	volume = volume + np.pi * (radius ** 2) * height
	return volume
	
def findTankHeight(radius, volume):
	height = volume / ((4 * np.pi * (radius ** 2)) + (np.pi * (radius ** 2)))
	return height
	
def findTankSurfaceArea(radius, height):
	surfaceArea = 4 * np.pi * (radius ** 2)
	surfaceArea = surfaceArea + 2 * np.pi * radius * height
	return surfaceArea
	
def circleSurfaceArea(radius):
	return np.pi * (radius ** 2)
	
def weightOfMetal(volume, density):
	return volume * density
	
shipRadius = 3.5
tankHeight = 4
materialDensity = const.TITANIUM_DENSITY
wallThickness = 0.002

deltaVRequiredForThreeHops = (findDeltaVRequiredForHop(50, 200, const.MOON_GRAV_AVG)) * 3
deltaVRequiredForMoonLanding = 2470
deltaVRequiredForMoonTakeoff = 2222
deltaVRequired = deltaVRequiredForMoonLanding + deltaVRequiredForThreeHops + deltaVRequiredForMoonTakeoff
print(deltaVRequired)

tankVolume = findTankVolume(shipRadius, tankHeight)
tankSurfaceArea = findTankSurfaceArea(shipRadius, tankHeight)
fuelTankWeight = weightOfMetal(tankSurfaceArea * wallThickness, const.TITANIUM_DENSITY)
bodyWeight = fuelTankWeight
engineWeight = 100 * 3
print(fuelTankWeight)
	
	
	
dry_mass = 15000.0
fuel_masses = np.arange(0, 90000)
deltaVs = []
Isp_vac = 360
for fuel_mass in fuel_masses:
	init_mass = fuel_mass + dry_mass
	deltaVs.append(findDeltaV(Isp_vac, init_mass, dry_mass))
	
plt.plot(fuel_masses, deltaVs)
plt.xlabel("fuel + oxidizer mass")
plt.ylabel("delta v")
plt.title("fuel mass vs delta v for Luna lander with module")
plt.show()

fuel_mass = 65000.0
oneMolOfMethaneMass = 0.016043
oneMolOfOxygenMass = .032
oneMolReactantMass = oneMolOfMethaneMass + oneMolOfOxygenMass + oneMolOfOxygenMass
molsOfReactant = fuel_mass / oneMolReactantMass
molsOfMethane = molsOfReactant
molsOfOxygen = molsOfReactant * 2
print(molsOfMethane)
print(molsOfOxygen)
methaneMass = molsOfMethane * oneMolOfMethaneMass
oxygenMass = molsOfOxygen * oneMolOfOxygenMass
print(methaneMass)
print(oxygenMass)
print(methaneMass + oxygenMass)
cubicMetersMethane = methaneMass / const.LIQUID_METHANE_DENSITY
cubicMetersOxygen = oxygenMass / const.LIQUID_OXYGEN_DENSITY
print(cubicMetersMethane)
print(cubicMetersOxygen)
cubicMetersFuelTank = cubicMetersMethane + cubicMetersOxygen
print(cubicMetersFuelTank)
tankHeight = findTankHeight(2, cubicMetersFuelTank)
print(tankHeight)
print(findTankVolume(2, tankHeight))



