from gym_env import WebotsAVEnv # Assume que o teu ficheiro é gym_env.py
import time

env = WebotsAVEnv()

for episodio in range(5): # Testa 5 resets
    print(f"--- Episódio {episodio + 1} ---")
    obs, info = env.reset()
    print("Ambiente resetado. Observação inicial recebida.")
    # Dá um tempo para observares o Webots
    # Simula alguns passos com ações aleatórias só para ver o ambiente a correr
    for _ in range(1000000000): # 50 passos
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        if terminated or truncated:
            break
    print("Pausa para observar o cenário no Webots (10 segundos)...")
    time.sleep(10) # Pausa para poderes ver o estado do mundo no Webots
env.close()
print("Teste de reset concluído.")