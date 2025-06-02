from controller import Supervisor
import random

# --- Configurações Iniciais ---
supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# --- Nomes DEF dos teus Objetos (AJUSTA ESTES NOMES!) ---
PEDESTRIAN_1_DEF_NAME = "PEDESTRIAN_1"  # Exemplo: o DEF name do teu nó de peão
OVERTAKING_CAR_DEF_NAME = "VEICULO_ULTRUPASSAR"  # Exemplo: o DEF name do teu carro de ultrapassagem
CROSSWALK_1_DEF_NAME = "PASSADEIRA_1"  # Exemplo: o DEF name da tua passadeira
PEDESTRIAN_2_DEF_NAME = "PEDESTRIAN_2"
CROSSWALK_2_DEF_NAME = "PASSADEIRA_2"

# --- Obter Referências aos Nós (Objetos) ---
try:
    pedestrian1_node = supervisor.getFromDef(PEDESTRIAN_1_DEF_NAME)
    overtaking_car_node = supervisor.getFromDef(OVERTAKING_CAR_DEF_NAME)
    crosswalk1_node = supervisor.getFromDef(CROSSWALK_1_DEF_NAME)
    pedestrian2_node = supervisor.getFromDef(PEDESTRIAN_2_DEF_NAME)
    crosswalk2_node = supervisor.getFromDef(CROSSWALK_2_DEF_NAME)
except Exception as e:
    print(f"Erro crucial: Não foi possível encontrar um ou mais nós. Verifica os nomes DEF: {e}")
    print(
        f"Nomes esperados: Pedestre='{PEDESTRIAN_1_DEF_NAME}', Pedestre='{PEDESTRIAN_2_DEF_NAME}', CarroUltrap='{OVERTAKING_CAR_DEF_NAME}', Passadeira='{CROSSWALK_1_DEF_NAME}' , Passadeira='{CROSSWALK_2_DEF_NAME}'")
    # Considera sair se os nós não forem encontrados, pois o resto falhará.
    # supervisor.simulationQuit(1)
    # Para depuração, vamos permitir que continue, mas com avisos.
    if 'pedestrian1_node' not in locals(): pedestrian1_node = None
    if 'overtaking_car_node' not in locals(): overtaking_car_node = None
    if 'crosswalk1_node' not in locals(): crosswalk1_node = None
    if 'pedestrian2_node' not in locals(): pedestrian2_node = None
    if 'crosswalk2_node' not in locals(): crosswalk2_node = None

if not all([pedestrian1_node, overtaking_car_node, crosswalk1_node, pedestrian2_node, crosswalk2_node]):
    print("AVISO: Um ou mais objetos principais não foram encontrados. A configuração dos cenários pode falhar.")

# --- Definir Posições das Zonas ---
# Define o centro [x, y, z] para cada zona onde um evento pode ocorrer.
# O valor Z deve ser o nível do solo. O Y pode ser 0 se a estrada for ao longo do eixo X.
ZONE_Z_LEVEL = 1
ZONE_POSITIONS = [
    [0.0,-400.0,ZONE_Z_LEVEL],  # Zona 1 -400 até -200
    [0.0,-200.0,ZONE_Z_LEVEL],  # Zona 2 -200 até 0
    [0.0,0.0,ZONE_Z_LEVEL],  # Zona 3 0 até 200
    [0.0,200.0,ZONE_Z_LEVEL]  # Zona 4 200 até 400
]
NUMBER_OF_ZONES = len(ZONE_POSITIONS)

# Posição de "arrumação" para objetos não utilizados (longe da área visível)
STORAGE_POSITION = [400.0, 0.0, ZONE_Z_LEVEL]

# --- Definir Tipos de Cenário ---
SCENARIO_OVERTAKING = "OVERTAKING"
SCENARIO_PED_AT_CROSSING = "PED_AT_CROSSING"
SCENARIO_EMPTY_CROSSING = "EMPTY_CROSSING"
SCENARIO_PED_JAYWALKING = "PED_JAYWALKING"
SCENARIO_NO_EVENT = "NO_EVENT"

# Lista dos cenários únicos que queres distribuir pelas zonas
# Garante que tens objetos suficientes se quiseres que certos cenários se repitam
# ou se mais de um cenário usa o mesmo objeto (ex: peão).
# Para esta versão, vamos assumir que cada um dos 4 cenários principais acontece uma vez.
# Se tiveres menos zonas que cenários, alguns não aparecerão.
# Se tiveres mais zonas, algumas ficarão como "NO_EVENT".
unique_scenarios_from_pdf = [
    SCENARIO_OVERTAKING,
    SCENARIO_PED_AT_CROSSING,
    SCENARIO_EMPTY_CROSSING,
    SCENARIO_PED_JAYWALKING
]


# --- Função Auxiliar para Mover um Nó ---
def move_node_to_position(node, position_xyz, rotation_xyzw=None):
    if node is None:
        # print(f"Aviso: Tentativa de mover um nó que não foi encontrado (None).")
        return

    translation_field = node.getField("translation")
    if translation_field:
        translation_field.setSFVec3f(position_xyz)
    # else:
    # print(f"Aviso: Nó '{node.getDef()}' não tem campo 'translation'.")

    if rotation_xyzw:  # rotation_xyzw é uma lista [axis_x, axis_y, axis_z, angle_radians]
        rotation_field = node.getField("rotation")
        if rotation_field:
            rotation_field.setSFRotation(rotation_xyzw)
        # else:
        # print(f"Aviso: Nó '{node.getDef()}' não tem campo 'rotation'.")

    # Para objetos dinâmicos que são teletransportados, pode ser necessário reiniciar a sua física
    # para evitar comportamentos estranhos. Se forem estáticos ou cinemáticos, geralmente não é preciso.
    # node.resetPhysics() # Usa com cuidado, pois afeta o objeto individualmente.


# --- Função Principal para Configurar os Cenários ---
def setup_world_scenarios():
    print("Configurando cenários para esta simulação...")

    # 1. Mover todos os objetos reutilizáveis para a posição de "arrumação"
    print("  Movendo objetos para a arrumação...")
    if pedestrian1_node: move_node_to_position(pedestrian1_node, STORAGE_POSITION)
    if overtaking_car_node: move_node_to_position(overtaking_car_node, STORAGE_POSITION)
    if crosswalk1_node: move_node_to_position(crosswalk1_node, STORAGE_POSITION)
    if pedestrian2_node: move_node_to_position(pedestrian2_node, STORAGE_POSITION)
    if crosswalk2_node: move_node_to_position(crosswalk2_node, STORAGE_POSITION)

    # 2. Gerar a lista de cenários para as zonas
    # Vamos garantir que cada um dos 4 cenários principais aparece uma vez, se houver zonas suficientes.
    # As restantes zonas serão "NO_EVENT".

    # Baralha as posições das zonas para que os cenários apareçam em locais diferentes
    shuffled_zone_indices = list(range(NUMBER_OF_ZONES))
    random.shuffle(shuffled_zone_indices)

    # Baralha os tipos de cenário únicos
    active_scenarios_for_run = list(unique_scenarios_from_pdf)  # Copia a lista
    random.shuffle(active_scenarios_for_run)

    final_zone_assignments = [SCENARIO_NO_EVENT] * NUMBER_OF_ZONES

    # Atribui os cenários únicos a zonas aleatórias
    # Limita ao número de cenários únicos ou ao número de zonas, o que for menor
    num_scenarios_to_place = min(len(active_scenarios_for_run), NUMBER_OF_ZONES)
    for i in range(num_scenarios_to_place):
        zone_idx = shuffled_zone_indices[i]
        final_zone_assignments[zone_idx] = active_scenarios_for_run[i]

    print(f"  Configuração de zonas: {final_zone_assignments}")

    # 3. Posicionar os objetos de acordo com os cenários atribuídos
    for i, zone_idx in enumerate(range(NUMBER_OF_ZONES)):  # Iterar pelas zonas na ordem original
        scenario_type = final_zone_assignments[zone_idx]
        zone_center_pos = ZONE_POSITIONS[zone_idx]

        print(f"  Processando Zona {zone_idx + 1} ({zone_center_pos}) com cenário: {scenario_type}")

        if scenario_type == SCENARIO_OVERTAKING:
            if overtaking_car_node:
                # Pode precisar de ajustar a posição exata
                # e a rotação para que o carro esteja orientado corretamente.
                # Esta é uma rotação padrão (sem rotação extra se o modelo já estiver orientado)
                car_rotation = [0, 0, 1, 1.57]
                move_node_to_position(overtaking_car_node, zone_center_pos, car_rotation)
                print(f"    Carro de ultrapassagem posicionado em {zone_center_pos}")
            else:
                print(f"    AVISO: Nó do carro de ultrapassagem não encontrado para {scenario_type}.")

        elif scenario_type == SCENARIO_PED_AT_CROSSING:
            if crosswalk1_node and pedestrian1_node:
                crosswalk_pos=[zone_center_pos[0],zone_center_pos[1],zone_center_pos[2]-1]
                move_node_to_position(crosswalk1_node, crosswalk_pos)
                # Posiciona o peão perto da passadeira (ex: na berma)
                # Ajusta estas coordenadas relativas conforme necessário
                ped_pos_relative_to_crosswalk = [zone_center_pos[0] + 7, zone_center_pos[1], zone_center_pos[2]]
                ped_rotation = [0, 0, 1, 3.1416]
                move_node_to_position(pedestrian1_node, ped_pos_relative_to_crosswalk, ped_rotation)
                print(f"    Passadeira em {crosswalk_pos}, Peão em {ped_pos_relative_to_crosswalk}")
            else:
                print(f"    AVISO: Nó da passadeira ou do peão não encontrado para {scenario_type}.")

        elif scenario_type == SCENARIO_EMPTY_CROSSING:
            if crosswalk2_node:
                crosswalk_pos = [zone_center_pos[0], zone_center_pos[1], zone_center_pos[2] - 1]
                move_node_to_position(crosswalk2_node, crosswalk_pos)
                print(f"    Passadeira vazia em {crosswalk_pos}")
            else:
                print(f"    AVISO: Nó da passadeira não encontrado para {scenario_type}.")

        elif scenario_type == SCENARIO_PED_JAYWALKING:
            if pedestrian2_node:
                # Posiciona o peão para atravessar fora da passadeira
                # Certifica-te que a passadeira (crosswalk_node) não está nesta zona se for o mesmo objeto.
                jaywalk_pos = [zone_center_pos[0] + 7, zone_center_pos[1],
                               zone_center_pos[2]]  # Ex: Na berma, pronto para atravessar
                jaywalk_rotation = [0, 0, 1, 3.1416]  # Ex: Virado para a estrada
                move_node_to_position(pedestrian2_node, jaywalk_pos, jaywalk_rotation)
                print(f"    Peão para 'jaywalking' em {jaywalk_pos}")
            else:
                print(f"    AVISO: Nó do peão não encontrado para {scenario_type}.")

        elif scenario_type == SCENARIO_NO_EVENT:
            print(f"    Zona {zone_idx + 1} sem evento.")
            # Os objetos já foram movidos para a arrumação se não foram usados.

    # supervisor.simulationSetMode(supervisor.SIMULATION_MODE_PAUSE) # Pausa para verificação
    # supervisor.simulationResetPhysics() # CUIDADO: Isto reinicia TODA a física, incluindo o teu carro RL.
    # Pode não ser o que queres aqui. É melhor se os objetos
    # forem cinemáticos ou se o seu próprio controlador os estabilizar.
    print("Configuração de cenários concluída.")


# --- Executar a Configuração no Início da Simulação ---
# No Webots, o script do supervisor é executado do início ao fim quando a simulação começa ou é reiniciada.
# Portanto, chamar a função de configuração aqui fará com que seja executada uma vez por cada "play" ou "reset".
setup_world_scenarios()

# --- Loop Principal da Simulação do Supervisor (pode fazer mais coisas) ---
# Este loop pode ser usado para monitorizar a simulação, mudar coisas dinamicamente
# com base no progresso do teu carro RL, ou simplesmente não fazer nada se a configuração
# inicial for suficiente.
while supervisor.step(timeStep) != -1:
    # Podes adicionar lógica aqui se o supervisor precisar de interagir durante a simulação.
    # Por exemplo, verificar se o teu carro passou por uma zona e limpar os objetos dessa zona.
    # Por agora, vamos mantê-lo simples.
    pass

# Código de limpeza (não é estritamente necessário aqui, pois o Webots trata disso ao fechar)
print("Fim do script do Supervisor.")