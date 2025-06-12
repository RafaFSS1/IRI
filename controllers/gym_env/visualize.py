from gym_env import WebotsAVEnv
from stable_baselines3 import PPO
import time

# --- Configuração da Avaliação ---
# Caminho para o modelo que quer testar
MODEL_PATH = "models/PPO/final_model_501760.zip"  # Altere para o nome do seu modelo!
NUM_EPISODES = 10

# --- Carregar o Modelo e o Ambiente ---
print(f"A carregar o modelo de: {MODEL_PATH}")
env = WebotsAVEnv()
model = PPO.load(MODEL_PATH, env=env)
print("Modelo e ambiente carregados.")

# --- Ciclo de Avaliação ---
for episode in range(NUM_EPISODES):
    print(f"\n--- A iniciar episódio de avaliação {episode + 1}/{NUM_EPISODES} ---")
    obs, info = env.reset()
    terminated = False
    truncated = False

    while not terminated and not truncated:
        # O modelo treinado prevê a melhor ação a tomar com base na observação
        # deterministic=True faz com que o agente não explore, usando apenas o que aprendeu.
        action, _states = model.predict(obs, deterministic=True)

        obs, reward, terminated, truncated, info = env.step(action)

        print(f"Ação: {action[0]:.2f}, Recompensa: {reward:.3f}")
       # Pausa para conseguir ver o que está a acontecer

print("\nAvaliação concluída.")
env.close()