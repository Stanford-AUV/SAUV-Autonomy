#!/usr/bin/python
import ms5837
import time
import numpy as np

# sensor = ms5837.MS5837_30BA() # Default I2C bus is 1 (Raspberry Pi 3)
# sensor = ms5837.MS5837_30BA(0) # Specify I2C bus
# sensor = ms5837.MS5837_02BA()
# sensor = ms5837.MS5837_02BA(7)
sensor = ms5837.MS5837(model=ms5837.MODEL_02BA, bus=7)  # Bar02, I2C bus 7
# sensor = ms5837.MS5837(model=ms5837.MS5837_MODEL_30BA, bus=0) # Specify model and bus

# We must initialize the sensor before reading it
if not sensor.init():
    print("Sensor could not be initialized")
    exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print(
    ("Pressure: %.2f atm  %.2f Torr  %.2f psi")
    % (
        sensor.pressure(ms5837.UNITS_atm),
        sensor.pressure(ms5837.UNITS_Torr),
        sensor.pressure(ms5837.UNITS_psi),
    )
)

print(
    ("Temperature: %.2f C  %.2f F  %.2f K")
    % (
        sensor.temperature(ms5837.UNITS_Centigrade),
        sensor.temperature(ms5837.UNITS_Farenheit),
        sensor.temperature(ms5837.UNITS_Kelvin),
    )
)

# freshwaterDepth = sensor.depth() # default is freshwater
# # sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
# # saltwaterDepth = sensor.depth() # No nead to read() again
# sensor.setFluidDensity(1000) # kg/m^3
# print(("Depth: %.3f m (freshwater)  %.3f m (saltwater)") % (freshwaterDepth, saltwaterDepth))

# fluidDensity doesn't matter for altitude() (always MSL air density)
print(
    ("MSL Relative Altitude: %.2f m") % sensor.altitude()
)  # relative to Mean Sea Level pressure in air

time.sleep(5)

# Spew readings

sample_idx = 0
max_samples = 10000
samples = np.zeros(max_samples)

while sample_idx < max_samples:
    if sensor.read():
        samples[sample_idx] = sensor.depth()  # in m
        print(sample_idx)
        sample_idx = sample_idx + 1

    else:
        print("Sensor read failed!")
        exit(1)


mean = np.mean(samples)
var = np.var(samples, ddof=1)

print(f"Samples: {samples}")
print(f"Mean Depth: {mean}")
print(f"Variance (Noise Covariance): {var}")
