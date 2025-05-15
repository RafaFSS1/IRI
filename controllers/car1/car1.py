from controller import GPS
from vehicle import Driver

driver = Driver()

time_step = int(driver.getBasicTimeStep())
MAX_SPEED = 65

driver.setSteeringAngle(0.0)
driver.setCruisingSpeed(MAX_SPEED)

gps = driver.getDevice("gps")
gps.enable(time_step)

