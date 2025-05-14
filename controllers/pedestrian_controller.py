from controller import Robot

# Cria instância do robô e liga ao Webots
robot = Robot()

# Obter o nome do robô (útil para debug)
print(f"Ligado ao robô: {robot.getName()}") # Isto funcionou antes

# Informação de diagnóstico
print(f"Tipo do objeto robot: {type(robot)}")
print(f"O objeto é uma instância de controller.Robot? {isinstance(robot, Robot)}")

print("\nAtributos e métodos disponíveis para o objeto 'robot':")
attributes = sorted(dir(robot))
for attr in attributes:
    print(f"  {attr}")

# Verificar especificamente a presença de métodos esperados da classe Node
print("\nVerificando métodos chave da classe Node:")
has_getField = hasattr(robot, 'getField')
print(f"  robot tem o método 'getField'? {has_getField}")

has_getType = hasattr(robot, 'getType') # Outro método de Node
print(f"  robot tem o método 'getType'? {has_getType}")

has_getPosition = hasattr(robot, 'getPosition') # Outro método de Node
print(f"  robot tem o método 'getPosition'? {has_getPosition}")

# Se getField realmente não existe, não podemos continuar com a lógica original
if not has_getField:
    print("\n--- ERRO DIAGNÓSTICO ---")
    print("O objeto 'robot' não parece ter o método 'getField'.")
    print("Isto é incomum e sugere um problema com a sua versão do Webots, instalação, ou configuração do ambiente Python.")
    print("Por favor, verifique:")
    print("  1. A sua versão do Webots (é muito antiga?).")
    print("  2. Se o Webots foi instalado corretamente e se as variáveis de ambiente (como PYTHONPATH) estão corretas, se aplicável.")
    print("  3. Se existe algum conflito de nomes com outras bibliotecas Python.")
    print("Considere reinstalar o Webots se o problema persistir.")
    exit()

# ---- O seu código original continua abaixo, se 'getField' existir ----
print("\nContinuando com a lógica original, pois 'getField' parece existir...")

# Obter o tempo de passo
time_step = int(robot.getBasicTimeStep()) # Isto funcionou antes

# Obter o campo "translation" do próprio robô
translation_field = robot.getField("translation")

# É uma boa prática verificar se o campo foi encontrado
if translation_field is None:
    print(f"Erro: Não foi possível encontrar o campo 'translation' para o robô '{robot.getName()}'.")
    print("Verifique se o nó do robô no Webots tem este campo (geralmente tem se for um objeto físico).")
    exit()

# Posição inicial
start_pos = translation_field.getSFVec3f()
target_z = start_pos[2] + 5.0  # destino no eixo Z

print(f"Posição inicial: {start_pos}")
print(f"Destino Z: {target_z}")

# Loop da simulação
while robot.step(time_step) != -1:
    pos = translation_field.getSFVec3f()
    if pos[2] < target_z:
        pos[2] += 0.01
        translation_field.setSFVec3f(pos)
    else:
        print(f"Pedestre chegou ao destino na posição: {pos}")
        break

print("Simulação terminada.")