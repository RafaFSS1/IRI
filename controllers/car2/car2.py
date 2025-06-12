from vehicle import Driver
import random
import math

driver = Driver()
time_step = int(driver.getBasicTimeStep())

# Constants
INITIAL_SPEED = 100
STEERING_ANGLE = 0.01
Counter_STEERING_ANGLE = 0.01
STEERING_DURATION = 150
Counter_STEERING_DURATION = 100
LOW_SPEED_DURATION_SECONDS = 10
LOW_SPEED = random.randint(50, 80)

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
start= False
event_over= False


CAR_NAME="VEICULO_TREINO"
MIN_PASS_DISTANCE_TO_TRIGGER = 30.0
car=driver.getFromDef(CAR_NAME)
overtake_pos=[1000.0,1000.0,1000.0]

while driver.step() != -1:
    if start:
        step_counter += 1

    if not start:
        car_pos = car.getField("translation").getSFVec3f()
        overtake_pos = driver.getSelf().getField("translation").getSFVec3f()
        dy = overtake_pos[1] - car_pos[1]
        if dy < MIN_PASS_DISTANCE_TO_TRIGGER:
            # Initial setup
            start = True
            driver.setCruisingSpeed(INITIAL_SPEED)
            print(INITIAL_SPEED)
            driver.setSteeringAngle(0.0)

    elif not overtake_started and start_step < step_counter:
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

    elif alignment_complete and not resumed_speed and not event_over:
        if step_counter - low_speed_step_start >= (LOW_SPEED_DURATION_SECONDS * 1000 / time_step):
            driver.setCruisingSpeed(INITIAL_SPEED)
            resumed_speed = True
            print(f"Resuming speed to {INITIAL_SPEED} m/s after {LOW_SPEED_DURATION_SECONDS} seconds.")

    if not event_over and driver.getSelf().getField("translation").getSFVec3f()[1] >= overtake_pos[1]+240.0:
        event_over = True
        print(f"VEICULO_ULTRAPASSAR: Ultrapassagem concluída. A desaparecer.")

    if event_over:
        driver.getSelf().getField("translation").setSFVec3f([100.0,0.0,1.0])
        driver.setCruisingSpeed(0)  # Comanda para parar (velocidade alvo 0)

    # Ensure straight driving before overtake
    if not overtake_started:
        driver.setSteeringAngle(0.0)
