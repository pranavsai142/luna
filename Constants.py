import VectorMath as vecm
import datetime
from skyfield.api import load

# Constants
G = 6.674e-11
EARTH_AVG_RADIUS = 6371 * 1000
EARTH_A_RADIUS = 6378.135 * 1000
EARTH_FLATTEN_CONST = 1/298.257
EARTH_MASS = 5.972e24
EARTH_STANDARD_GRAV = G * EARTH_MASS
EARTH_GRAV_SEA = EARTH_STANDARD_GRAV / (EARTH_AVG_RADIUS**2)
EARTH_GRAV_AVG = 9.80665

MOON_AVG_RADIUS = 1737.4 * 1000
MOON_A_RADIUS = 1738.1 * 1000
MOON_FLATTEN_CONST = 0.0012
MOON_MASS = .07346e24
MOON_STANDARD_GRAV = G * MOON_MASS
MOON_GRAV_AVG = 1.625

delta_t = datetime.timedelta(microseconds=10000)
hstep = datetime.timedelta(seconds=10)
hstep_total_seconds = hstep.total_seconds()
simulationduration = 10000

	
ECI_X = vecm.vector(1,0,0)
ECI_Y = vecm.vector(0,1,0)
ECI_Z = vecm.vector(0,0,1)

# kg/m^3
TITANIUM_DENSITY = 4506
LIQUID_OXYGEN_DENSITY = 1141
LIQUID_METHANE_DENSITY = 422.8

planets = load('de421.bsp')
earth = planets['earth']
moon = planets['moon']
sun = planets['sun']
ts = load.timescale()

RCS_TIME_BUFFER = datetime.timedelta(seconds = 120)