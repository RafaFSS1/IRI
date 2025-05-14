from controller import Supervisor
import math

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

pedestrian_node = robot.getFromDef("PEDESTRIAN_1")

if pedestrian_node is None:
    print("Erro: não encontrei o nó PEDESTRIAN_1")
    exit()

translation_field = pedestrian_node.getField("translation")
rotation_field = pedestrian_node.getField("rotation")

while robot.step(time_step) != -1:
    pos = translation_field.getSFVec3f()
    rot = rotation_field.getSFRotation()  # [x, y, z, angle]

    # O pedestre está rotacionado no eixo Z → direção é no plano XY
    angle = rot[3]

    # Move na direção horizontal conforme a rotação
    dir_x = math.cos(angle)
    dir_y = math.sin(angle)

    # Aplica movimento na horizontal (X, Y), mantém Z fixo
    pos[0] += dir_x * 0.01
    pos[1] += dir_y * 0.01  # Y neste caso é o eixo horizontal (não altura)

    translation_field.setSFVec3f(pos)