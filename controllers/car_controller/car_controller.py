from controller import Supervisor

robot = Supervisor()
time_step = int(robot.getBasicTimeStep())

car_node = robot.getFromDef("Veiculo_ultrupassar")

if car_node is None:
    print("Erro: não encontrei o carro EGO_CAR")
    exit()

translation_field = car_node.getField("translation")

while robot.step(time_step) != -1:
    pos = translation_field.getSFVec3f()

    # Obtemos a matriz de orientação 3x3 (lista com 9 valores)
    orientation = car_node.getOrientation()

    # O vetor da frente do carro está na 3ª coluna da matriz
    dir_x = orientation[2]  # X
    dir_y = orientation[8]  # Y

    # Mantém a altura original (não mexe em Y)
    new_pos = [0, 0, 0]
    new_pos[0] = pos[0] + dir_x * 0.02
    new_pos[2] = pos[2]               # mantém altura constante
    new_pos[1] = pos[1] + dir_y * 0.02

    translation_field.setSFVec3f(new_pos)