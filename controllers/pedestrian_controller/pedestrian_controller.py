from controller import Supervisor
import math

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

CAR_NAME= "VEICULO_TREINO"
TRIGER_DISTANCE = 50.0

translation_field = robot.getSelf().getField("translation")
rotation_field = robot.getSelf().getField("rotation")

car=robot.getFromDef(CAR_NAME)

has_started_walking = False

while robot.step(time_step) != -1:

    pos = translation_field.getSFVec3f()
    rot = rotation_field.getSFRotation()  # [x, y, z, angle]

    if not has_started_walking:
        car_pos = car.getField("translation").getSFVec3f()
        dy = pos[1] - car_pos[1]
        if dy < TRIGER_DISTANCE:
            print("Peão a andar")
            has_started_walking = True

    if has_started_walking:
        # O pedestre está rotacionado no eixo Z → direção é no plano XY
        angle = rot[3]

        # Move na direção horizontal conforme a rotação
        dir_x = math.cos(angle)
        dir_y = math.sin(angle)

        # Aplica movimento na horizontal (X, Y), mantém Z fixo
        pos[0] += dir_x * 0.02
        pos[1] += dir_y * 0.02  # Y neste caso é o eixo horizontal (não altura)

        translation_field.setSFVec3f(pos)