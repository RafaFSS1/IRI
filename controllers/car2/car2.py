from vehicle import Driver
import random

driver = Driver()
time_step = int(driver.getBasicTimeStep())

# Constants
INITIAL_SPEED = 80
STEERING_ANGLE = 0.01
Counter_STEERING_ANGLE = 0.01
STEERING_DURATION = 200
Counter_STEERING_DURATION = 160
LOW_SPEED_DURATION_SECONDS = 10
LOW_SPEED = random.randint(20, 45)

# Initial setup
driver.setCruisingSpeed(INITIAL_SPEED)
driver.setSteeringAngle(0.0)

# Delay before starting overtake
start_delay = random.uniform(5, 8)
start_step = int(start_delay * 1000 / time_step)

# Step counters and flags
step_counter = 0
overtake_started = False
turning_phase = False
counter_steering_phase = False
alignment_complete = False
low_speed_step_start = None
resumed_speed = False

while driver.step() != -1:
    step_counter += 1

    if not overtake_started and step_counter >= start_step:
        overtake_started = True
        turning_phase = True
        turn_start_step = step_counter
        print(f"Starting lane change at step {step_counter}")
        driver.setSteeringAngle(STEERING_ANGLE)

    elif turning_phase:
        if step_counter - turn_start_step >= STEERING_DURATION:
            turning_phase = False
            counter_steering_phase = True
            counter_turn_start_step = step_counter
            driver.setSteeringAngle(-Counter_STEERING_ANGLE)
            print("Counter-steering to re-align.")

    elif counter_steering_phase:
        if step_counter - counter_turn_start_step >= Counter_STEERING_DURATION:
            counter_steering_phase = False
            alignment_complete = True
            driver.setSteeringAngle(0.0)
            driver.setCruisingSpeed(LOW_SPEED)
            low_speed_step_start = step_counter
            print(f"Finished alignment. Cruising at {LOW_SPEED} m/s.")

    elif alignment_complete and not resumed_speed:
        if step_counter - low_speed_step_start >= (LOW_SPEED_DURATION_SECONDS * 1000 / time_step):
            driver.setCruisingSpeed(INITIAL_SPEED)
            resumed_speed = True
            print(f"Resuming speed to {INITIAL_SPEED} m/s after {LOW_SPEED_DURATION_SECONDS} seconds.")

    # Ensure straight driving before overtake
    if not overtake_started:
        driver.setSteeringAngle(0.0)
