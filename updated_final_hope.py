

# Converted to .py using https://pypi.org/project/ipynb-py-convert/

# %%
"""
### Deimos Sim
"""



# %%
# Import all required packages from their relevant sources

from rocketpy import Environment, SolidMotor, Rocket, Flight
import datetime
import numpy as np
import pandas as pd
import json
from sympy import *

# %%
"""
#### Pre Flight Checklist
* Get weather data
    * use weather API when internet access available
    * store backup weather forecast for the launch day
* Review the following with the lead engineer for rocket under simulation
* Check motor instantiation
    * motor thrust curve
    * motor grain height
    * motor inner radius
    * motor outer radius
    * number of grains
    * burn time
* Check physical measurements
    * weight rocket components
    * measure rocket dimensions
    * find centre of mass
    * potentially find moment of inertias
* Check drag curves
    * run simulation with OpenRocket drag curves
    * run a simulation with RASAero II drag curves
* Confirm with lead engineer for rocket that the target apogee is within acceptable range
* If any changes need to be made to simulation the following need to be rechecked
    * input balast into open rocket and run rough simulation
    * adjust:
        * LOADED_ROCKET
        * UNLOADED_ROCKET
        * INERTIA_I
        * INERTIA_Z
        * LOADED_CG
        * UNLOADED_CG
        * CG_TO_NOZZLE
        * MOTOR_CG_DIST_FROM_CONE
        * CG_TO_MOTOR_CG
        * CONE: UNLOADED_CG
        * CONE: CONE_DIST_TO_CG
        * FINS: DIST_TO_CG

"""

# %%
from google.colab import drive
drive.mount('/content/drive')

# %%


# %%
"""
#### Initialise the Environment
"""

# %%
"""
## Environment Information
railLength: length of rail that the rocket is stabilised on for vertical launch in metres
date (2022, 4, 23, 16) is read is the 23rd of April 2022 at 4pm
timeZone: timezone of launch site (Serpentine Launch Site: Australia/Melbourne)
latitude: latitude of launch site (Serpentine Launch Site: -36.50066)
longitude: longitude of launch site (Serpentine Launch Site: 144.01692)
elevation: elevation of launch site in metres (Serpentine Launch Site: 120m)
"""

# %%
"""
# Initialising the Environment Object Using Manually Collected Weather Data
"""

# %%
# EXAMPLE using manual weather data for Sunday 15th May 2022
# Env.setAtmosphericModel(
#     type="CustomAtmosphere",
#     pressure=101030,          # Pa
#     temperature=291.65,       # K (Kelvin)
#     wind_u=[(0, 0)],          # direction east
#     wind_v=[(0, 6.67)],       # direction north (altitude m, windspeed m/s)
# )

# %%
"""
# Initialising the Environment Object Using Automatically Collected Weather Data from Online Sources
"""

# %%
# Note: Using last year's information from the day that we launched. Predicted to be similar this year

# Initilaise the environment with the rail length, and location & elevation information
Env = Environment(railLength=5.2)
Env.setLocation(32.990254, -106.974998)
Env.setElevation(1401)

# Open the file and load the data into a JSON object
fileName = '/content/drive/MyDrive/rocket/rocketpy-sims/Deimos 2023/utils/weatherData/fri_1pm.json'
fileData = open(fileName, "r", encoding="utf-8")
data = json.load(fileData)
fileData.close()

# Set the environment atmospheric model using the data from that JSON object
Env.setAtmosphericModel(type='CustomAtmosphere', temperature=data['data']['temperature'], pressure=data['data']['pressure'], wind_u=data['data']['wind_u'], wind_v=data['data']['wind_v'])

# %%
"""
# Initialising the Environment Class Using Online Data Source (GFS)
"""

# %%
# # Initialise the Environment variable with the rail length (m), time zone, latitude, longitude, elevation (m), date time object of the launch time
# Env = Environment(
#         railLength=5.2,
#         timeZone='America/Denver',
#         latitude=32.990254,
#         longitude=-106.974998,
#         elevation=1401,
#         date=datetime(2023, 6, 20, 12)
#     )

# # Set atmospheric model to be generating a forecast based on data from the GFS
# Env.setAtmosphericModel(type="Forecast", file="GFS")

# %%
# Print out the environment information
Env.info()

# %%
"""
#### Create Motor
"""

# %%
# The total length of the motor in metres
MOTOR_LENGTH = 1.239 #m

# The number of grains in the rocket motor
NUM_GRAINS = 6

# The outer radius of a grain in the rocket motor in metres
# We should be taking measurements of all of the outer diameters of all of the grains, and taking the average to get the most representative value,
# then divide by two to get the radius
# Here we only have one:
OUTER_GRAIN_DIAMETER = 0.088 # m
OUTER_GRAIN_RADIUS = OUTER_GRAIN_DIAMETER/2  # m

# The outer radius of a grain in the rocket motor in metres
# We should be taking measurements of all of the inner diameters of all of the grains, and taking the average to get the most representative value,
# then divide by 2 to get the radius
INNER_DIAMETER_LIST = [0.058, 0.058, 0.058, 0.058, 0.058, 0.058]    # m
INNER_DIAMETER_AVG =  sum(INNER_DIAMETER_LIST)/len(INNER_DIAMETER_LIST)  # m
INNER_GRAIN_RADIUS = INNER_DIAMETER_AVG/2  # m

# The height of a single grain in the motor in metres
# We should be taking measurements of all of the inner diameters of all of the grains, and taking the average to get the most representative value
# Here we only have one:
GRAIN_HEIGHT = 0.20483 #m

# The mass of the propellant in the motor, in kilograms
# We should be taking the mass of all of the motor grains, then adding them together to get the whole propellant mass
# Here we just have the total propellant mass from the manufacturer:
PROPELLANT_MASS = 9.021 # kg

## NOT REQUIRED but always good to include information if available:
# The radius of the motor nozzle in metres (outer part of the nozzle)
# NOZZLE_DIAMETER = 0.0275  # m
# NOZZLE_RADIUS = NOZZLE_DIAMETER/2  # m

# The radius of the throat of the motor nozzle in metres (inner part of the nozzle)
# THROAT_DIAMETER = 0.0146  # m
# THROAT_RADIUS = THROAT_DIAMETER/2  # m

# %%
# Print out the motor characteristics
print("-------------- Motor Characteristics --------------")
print("Motor length (m)", MOTOR_LENGTH)
print("Average grain height (m)", GRAIN_HEIGHT)
print("Propellant mass (kg)", PROPELLANT_MASS)
print("Outer diameter (m)", OUTER_GRAIN_DIAMETER)
print("Outer radius (m), ", OUTER_GRAIN_RADIUS)
print("Inner diameter (m)", INNER_DIAMETER_AVG)
print("Inner radius (m)", INNER_GRAIN_RADIUS)
# print("Nozzle radius (m)", NOZZLE_RADIUS)
# print("Throat radius (m)", THROAT_RADIUS)


# Pass through the information that has been gathered to a SolidMotor object
N5800 = SolidMotor(
   thrustSource= "/content/drive/MyDrive/rocket/rocketpy-sims/Deimos 2023/motors/Cesaroni_20146N5800-P.eng",
   burnOut=3.5,
   grainNumber=NUM_GRAINS,
   grainDensity=(PROPELLANT_MASS/NUM_GRAINS)/(np.pi*(OUTER_GRAIN_RADIUS**2)*GRAIN_HEIGHT - np.pi*(INNER_GRAIN_RADIUS**2)*GRAIN_HEIGHT),
   grainOuterRadius=OUTER_GRAIN_RADIUS,
   grainInitialInnerRadius=INNER_GRAIN_RADIUS,
   grainInitialHeight=GRAIN_HEIGHT,
   # throatRadius=THROAT_RADIUS,
   # nozzleRadius=NOZZLE_RADIUS,
   interpolationMethod="linear",
)

# %%
# Print out information about the motor
N5800.info()

# %%
"""
#### Drag Curve
"""

# %%
# Remove duplicate Mach number as a function of axial drag coeficient
# NOTE: RocketPy requires uniqueness in the Mach number and resulting axial drag coefficient. Here, we are removing all duplicates to ensure
# that only one drag coefficient is being passed through for each Mach number

POWER_OFF_DRAG="/content/drive/MyDrive/rocket/rocketpy-sims/Deimos 2023/drag/power-off-drag-ork-N5800.csv"
POWER_ON_DRAG="/content/drive/MyDrive/rocket/rocketpy-sims/Deimos 2023/drag/power-on-drag-ork-N5800.csv"

files = [POWER_ON_DRAG, POWER_OFF_DRAG]

# Iterate through each of the drag files, and ensure that all duplicate Mach number lines are removed prior to being passed to the Rocket object
for file in files:
    df = pd.read_csv(file)
    df.columns = ['MACH', 'CD']
    df = df.drop_duplicates(subset='MACH')
    df.to_csv(file, index=False, header=False)


# %%
"""
#### Create Rocket
"""

# %%
# Motor characteristics
# NOTE: add motor casing mass to mass of rocket without propellant in order to get the unloaded rocket mass
MOTOR_MASS = 14.826     # kg
MOTOR_CASING_MASS = MOTOR_MASS - PROPELLANT_MASS   # kg

# ------- Rocket characteristics ---------------------

# Mass
LOADED_ROCKET = 33.08 # kg
UNLOADED_ROCKET = LOADED_ROCKET - MOTOR_MASS + MOTOR_CASING_MASS  # kg

# Radius
DIAMETER = 15.4/100  # m - note that this is the diameter of the widest point of the rocket body
RADIUS = DIAMETER/2  # m

# Length
TOTAL_LENGTH = 398/100  # m


# NOTE: need to manually calculate this for unloaded rocket - use the data of the rocket after burnout
# Unloaded rocket lateral (perpendicular to axis of symmetry) moment of inertia (without propelant)
# AKA longitudinal moment of inertia # yaw axis
unloaded_i = 25.987
UNLOADED_INERTIA_I = unloaded_i # yaw axis moment after motor burnout



# Unloaded rocket axial moment of inertia (without propellant)
# AKA rotational moment of inertia # roll axis
unloaded_z = 0.049
UNLOADED_INERTIA_Z = unloaded_z # roll axis moment after motor burnout

UNLOADED_CG = 230/100

# NOTE: Taking origin of rocket as unloaded centre of mass
# Using unloaded CG, calculate the:
# The entre of gravity to the nozzle
CG_TO_NOZZLE = UNLOADED_CG - TOTAL_LENGTH  # -m
# The centre of gravity of the motor to the cone
MOTOR_CG_DIST_FROM_CONE = TOTAL_LENGTH - MOTOR_LENGTH/2  # m
# The centre of gravity to the motor's centre of gravity
CG_TO_MOTOR_CG = UNLOADED_CG - MOTOR_CG_DIST_FROM_CONE  # -m

print("Radius: " + str(RADIUS) + " m")
print("Mass: " + str(UNLOADED_ROCKET) + " kg")
print("Longitudinal Inertia (interiaI): " + str(UNLOADED_INERTIA_I) + " kgm^2")
print("Rotational Inertia (interiaZ): " + str(UNLOADED_INERTIA_Z) + " kgm^2")
print("Distance from CG to rocket nozzle: " + str(CG_TO_NOZZLE) + " m")
print("Distance from CG to motor CG: " + str(CG_TO_MOTOR_CG) + " m\n")

# Initialise rocket
Deimos = Rocket(
    motor=N5800,
    radius=RADIUS,
    mass=UNLOADED_ROCKET,
    inertiaI=UNLOADED_INERTIA_I,
    inertiaZ=UNLOADED_INERTIA_Z,
    distanceRocketNozzle=CG_TO_NOZZLE,
    distanceRocketPropellant=CG_TO_MOTOR_CG,
    powerOffDrag=POWER_OFF_DRAG,
    powerOnDrag=POWER_ON_DRAG,
)

# %%
# Set the location of the rain buttons from the nose cone of the rocket
Deimos.setRailButtons([UNLOADED_CG - (TOTAL_LENGTH-1.479), UNLOADED_CG - (TOTAL_LENGTH - 0.94)])

# %%
"""
####  Aerodynamic Surfaces
"""

# %%
# Cone measurements in m
CONE_LENGTH = 0.75  # m
CONE_KIND = 'ogive'
CONE_DIST_TO_CG = UNLOADED_CG - CONE_LENGTH   # from cone base to CG in m

# Fin measurements in m
FIN_NUM = 4
SPAN_HEIGHT = 0.141  # m
FIN_ROOT_CHORD = 0.344  # m
FIN_TIP_CHORD = 0.012  # m
FIN_OFFSET = 0.004  # m
TOP_OF_FIN = TOTAL_LENGTH-(FIN_ROOT_CHORD+FIN_OFFSET)
DIST_TO_CG = UNLOADED_CG - TOP_OF_FIN  # m

# Add the aerodynamic surfaces to the rocket
# Nose Cone
NoseCone = Deimos.addNose(length=CONE_LENGTH, kind=CONE_KIND, distanceToCM=CONE_DIST_TO_CG)

# Fin Set
FinSet = Deimos.addTrapezoidalFins(
    FIN_NUM, span=SPAN_HEIGHT, rootChord=FIN_ROOT_CHORD, tipChord=FIN_TIP_CHORD, distanceToCM=DIST_TO_CG
)

# Boat Tail - need to get!
# Tail = BigBlue.addTail(
#     topRadius=0.0635, bottomRadius=0.0435, length=0.060, distanceToCM=-1.194656
# )

# %%
"""
#### Parachutes
"""

# %%
# Define the parachute trigger that deploys the drogue parachute at apogee
def drogueTrigger(p, y):
    # p = pressure
    # y = [x, y, z, vx, vy, vz, e0, e1, e2, e3, w1, w2, w3]
    # activate drogue when vz < 0 m/s.
    return True if y[5] < 0 else False

# Define the parachute trigger that deploys the main parachute at a certain height
def mainTrigger(p, y):
    # p = pressure
    # y = [x, y, z, vx, vy, vz, e0, e1, e2, e3, w1, w2, w3]
    # activate main when vz < 0 m/s and z < 800 m.
    return True if y[5] < 0 and y[2] < 500 else False

# Define the drag coefficients and radii of the main and drogue parachutes
DRAG_COEFF_DROGUE = 1.55
DRAG_COEFF_MAIN = 2.20
DROGUE_RADIUS = 0.61
MAIN_RADIUS = 1.22

# Add the parachutes to the rocket
Main = Deimos.addParachute(
    "Main",
    CdS=DRAG_COEFF_MAIN*(MAIN_RADIUS)**2*np.pi, #Times reference area
    trigger=mainTrigger,
    samplingRate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

Drogue = Deimos.addParachute(
    "Drogue",
    CdS=DRAG_COEFF_DROGUE*(DROGUE_RADIUS)**2*np.pi, # Times reference area
    trigger=drogueTrigger,
    samplingRate=105,
    lag=1.5,
    noise=(0, 8.3, 0.5),
)

# %%
"""
#### Flight Simulation
"""

# %%
# need to have Deimos.setRailButtons(...) for this to work

# Create a flight object of the rocket
TestFlight = Flight(rocket=Deimos, environment=Env, inclination=88, heading=0)

# %%
# Get the information about the rocket's flight
TestFlight.apogee
TestFlight.staticMargin

# %%
analysis_parameters = {
    # Mass Details
    "rocketMass": (
        Deimos.mass,
        0.001,
    ),  # Rocket's dry mass (kg) and its uncertainty (standard deviation)
    # Propulsion Details - run help(SolidMotor) for more information
    "impulse": (N5800.totalImpulse, 35.3),  # Motor total impulse (N*s)
    "burnOut": (N5800.burnOutTime, 0),  # Motor burn out time (s)
    # Motor's nozzle radius (m)
    "nozzleRadius": (N5800.nozzleRadius, 0.5 / 1000),
    "throatRadius": (
        N5800.throatRadius,
        0.5 / 1000,
    ),  # Motor's nozzle throat radius (m)
    "grainSeparation": (
        N5800.grainSeparation,
        1 / 1000,
    ),  # Motor's grain separation (axial distance between two grains) (m)
    "grainDensity": (N5800.grainDensity, 50),  # Motor's grain density (kg/m^3)
    "grainOuterRadius": (
        N5800.grainOuterRadius,
        0.375 / 1000,
    ),  # Motor's grain outer radius (m)
    "grainInitialInnerRadius": (
        N5800.grainInitialInnerRadius,
        0.375 / 1000,
    ),  # Motor's grain inner radius (m)
    "grainInitialHeight": (
        N5800.grainInitialHeight,
        1 / 1000,
    ),  # Motor's grain height (m)
    # Aerodynamic Details - run help(Rocket) for more information
    "inertiaI": (
        Deimos.inertiaI,
        0.03675,
    ),  # Rocket's inertia moment perpendicular to its axis (kg*m^2)
    "inertiaZ": (
        Deimos.inertiaZ,
        0.00007,
    ),  # Rocket's inertia moment relative to its axis (kg*m^2)
    "radius": (Deimos.radius, 0.001),  # Rocket's radius (kg*m^2)
    "distanceRocketNozzle": (
        Deimos.distanceRocketNozzle,
        0.001,
    ),  # Distance between rocket's center of dry mass and nozzle exit plane (m) (negative)
    "distanceRocketPropellant": (
        Deimos.distanceRocketPropellant,
        0.001,
    ),  # Distance between rocket's center of dry mass and and center of propellant mass (m) (negative)
    "powerOffDrag": (
        1 / 1.05,
        0.033,
    ),  # Multiplier for rocket's drag curve. Usually has a mean value of 1 and a uncertainty of 5% to 10%
    "powerOnDrag": (
        1 / 1.05,
        0.033,
    ),  # Multiplier for rocket's drag curve. Usually has a mean value of 1 and a uncertainty of 5% to 10%
    "noseLength": (CONE_LENGTH, 0.001),  # Rocket's nose cone length (m)
    "noseDistanceToCM": (
        UNLOADED_CG - TOTAL_LENGTH,
        0.001,
    ),  # Axial distance between rocket's center of dry mass and nearest point in its nose cone (m)
    "finSpan": (SPAN_HEIGHT, 0.0005),  # Fin span (m)
    "finRootChord": (FIN_ROOT_CHORD, 0.0005),  # Fin root chord (m)
    "finTipChord": (FIN_TIP_CHORD, 0.0005),  # Fin tip chord (m)
    "finDistanceToCM": (
        FIN_OFFSET,
        0.001,
    ),  # Axial distance between rocket's center of dry mass and nearest point in its fin (m)
    # Launch and Environment Details - run help(Environment) and help(Flight) for more information
    "inclination": (
        88,
        1,
    ),  # Launch rail inclination angle relative to the horizontal plane (degrees)
    "heading": (0, 2),  # Launch rail heading relative to north (degrees)
    "railLength": (5.2, 0.0005),  # Launch rail length (m)
    # Members of the ensemble forecast to be used
    #"ensembleMember": list(range(10)),
    # Parachute Details - run help(Rocket) for more information
    "CdSDrogue": (
        DRAG_COEFF_DROGUE * (DROGUE_RADIUS) ** 2 * np.pi,
        0.07,
    ),  # Drag coefficient times reference area for the drogue chute (m^2)
    "CdSMain": (
        DRAG_COEFF_MAIN * (MAIN_RADIUS) ** 2 * np.pi,
        0.07,
    ),  # Drag coefficient times reference area for the main chute (m^2)
    "lag_rec": (
        1,
        0.5,
    ),  # Time delay between parachute ejection signal is detected and parachute is inflated (s)
    # Electronic Systems Details - run help(Rocket) for more information
    "lag_se": (
        0.73,
        0.16,
    ),  # Time delay between sensor signal is received and ejection signal is fired (s)
}

# %%
"""
Define a objective funtion

it consider only launch stage(the journey after reaching apogee). The meric paraemter inlcuding distance CG and CP, horizontal distance and apogee. the varing parameter is ballast weight(0 to 10g) and poistion from nose to body cube. The inital weight for apogee, stabilityMargin and hozrion distance will be 50, 30 and 20 ( sum of weight will 100)
"""

# %%
weight_apg= 50
weight_hzor=20
weight_CPACG=30

target_appogee=91440
target_hozri=0
target_stability=1

# initalization matrix

apgee_np= np.empty((2,2))
stability=np.empty((2,2))
horiz_np=np.empty((2,2))




# %%
"""
Here function is tuning ballast weight and position. Calcualting longitudual inertia will consider using cylinder as simple case.  And center of mass will calcauted as well
"""

# %%
def Rokcetweigh_len_benchmark(Ballastweigh, position):
  MOTOR_MASS = 14.826   # kg
  MOTOR_CASING_MASS = MOTOR_MASS - PROPELLANT_MASS
  LOADED_ROCKET = 20.2+Ballastweigh # kg
  UNLOADED_ROCKET = LOADED_ROCKET - MOTOR_MASS + MOTOR_CASING_MASS  # kg

  DIAMETER = 15.4/100  # m - note that this is the diameter of the widest point of the rocket body
  RADIUS = DIAMETER/2
  UNLOADED_CG = Cen_mass_cal(Ballastweigh,position)
  TOTAL_LENGTH = 398/100
  unloaded_i = 0.049*0.5+UNLOADED_ROCKET*(1/12)*TOTAL_LENGTH*TOTAL_LENGTH*1.0767 # there is offset paraemter for reducing error from curent function
  UNLOADED_INERTIA_I = unloaded_i # yaw axis moment after motor burnout

  unloaded_z = 0.049
  UNLOADED_INERTIA_Z = unloaded_z # roll axis moment after motor burnout
  # UNLOADED_CG = 230/100


  MOTOR_CG_DIST_FROM_CONE = TOTAL_LENGTH - MOTOR_LENGTH/2
  CG_TO_MOTOR_CG = UNLOADED_CG - MOTOR_CG_DIST_FROM_CONE
  CONE_LENGTH = 0.75  # m
  CONE_KIND = 'ogive'
  CONE_DIST_TO_CG = UNLOADED_CG - CONE_LENGTH   # from cone base to CG in m
  Deimos = Rocket(
    motor=N5800,
    radius=RADIUS,
    mass=UNLOADED_ROCKET,
    inertiaI=UNLOADED_INERTIA_I,
    inertiaZ=UNLOADED_INERTIA_Z,
    distanceRocketNozzle=CG_TO_NOZZLE,
    distanceRocketPropellant=CG_TO_MOTOR_CG,
    powerOffDrag=POWER_OFF_DRAG,
    powerOnDrag=POWER_ON_DRAG,
  )
  FIN_NUM = 4
  SPAN_HEIGHT = 0.141  # m
  FIN_ROOT_CHORD = 0.344  # m
  FIN_TIP_CHORD = 0.012  # m
  FIN_OFFSET = 0.004  # m
  TOP_OF_FIN = TOTAL_LENGTH-(FIN_ROOT_CHORD+FIN_OFFSET)
  DIST_TO_CG = UNLOADED_CG - TOP_OF_FIN  # m
  NoseCone = Deimos.addNose(length=CONE_LENGTH, kind=CONE_KIND, distanceToCM=CONE_DIST_TO_CG)
  FinSet = Deimos.addTrapezoidalFins(
    FIN_NUM, span=SPAN_HEIGHT, rootChord=FIN_ROOT_CHORD, tipChord=FIN_TIP_CHORD, distanceToCM=DIST_TO_CG
  )
  TestFlight = Flight(rocket=Deimos, environment=Env, inclination=88, heading=0,terminateOnApogee=True)

  TestFlight.apogee



# %%
"""
Design function for find center of mass
"""

# %%


Nose_cone_w=0.642 # mass: unit=kg
Nose_cone_l=38.5 # distance from nose to current objectve center of mass : unit: cm

Body_tub1_w=1.379
Body_tub1_l=65.8

Body_tub2_w=0.839
Body_tub2_l=85.8

Body_tub3_w=0.191
Body_tub3_l=90.8

Body_tub4_w=0.394
Body_tub4_l=100.8

Body_tub5_w=1.73
Body_tub5_l=168.3

Body_tub6_w=1.49
Body_tub6_l=221.8

Fin_w=0.805
Fin_l=392.5

BoatTail_w=0.1
BoatTail_l=407.5

Drog_chute_w=0.039
Drog_chute_l=115

Avonic_w=2
Avonic_l=159

Main_chute_w=0.12
Main_chute_l=220

Motor_w=5.4
Motor_l=340

Ring1_w=0.28
Ring1_l=408.5

Ring2_w=0.114
Ring2_l=375.5

Ring3_w=0.114
Ring3_l=351.5

Ring4_w=0.114
Ring4_l=325.5


def Cen_mass_cal(W, L):
  Z=symbols("Z") # set center of mass distance as Z


  equation=Nose_cone_w*(Z-Nose_cone_l)+Body_tub1_w*(Z-Body_tub1_l)+Body_tub2_w*(Z-Body_tub2_l)+Body_tub3_w*(Z-Body_tub3_l)+Body_tub4_w*(Z-Body_tub4_l)+Body_tub5_w*(Z-Body_tub5_l)+Body_tub6_w*(Z-Body_tub6_l)+Fin_w*(Z-Fin_l)+BoatTail_w*(Z-BoatTail_l)+Drog_chute_w*(Z-Drog_chute_l)+Avonic_w*(Z-Avonic_l)+Main_chute_w*(Z-Main_chute_l)+Motor_w*(Z-Motor_l)+Ring1_w*(Z-Ring1_l)+Ring2_w*(Z-Ring2_l)+Ring3_w*(Z-Ring3_l)+Ring4_w*(Z-Ring4_l)

  sol=solve(equation)
  return sol





# %%
sol=Cen_mass_cal(4,65)
print(sol)

# %%
