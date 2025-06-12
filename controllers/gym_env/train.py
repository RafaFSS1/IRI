import os
from gym_env import WebotsAVEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from stable_baselines3.common.logger import configure

# --- Configurações do Treino ---
models_dir = "models/PPO"
logdir = "logs"

# --- !! NOVA SECÇÃO PARA CONTINUAR O TREINO !! ---
# Coloque aqui o caminho para o modelo que quer continuar a treinar.
# Se quiser começar um treino do ZERO, deixe esta variável como None.
# Exemplo: "models/PPO/final_model_100000.zip" ou "models/PPO/av_driver_model_80000_steps.zip"
LOAD_MODEL_PATH = None  # <-- ALTERE AQUI

# --- Criação do Ambiente ---
print("A criar o ambiente WebotsAVEnv para treino...")
env = WebotsAVEnv()
env.reset()
print("Ambiente criado.")

# --- Configuração do Modelo (PPO) ---
if LOAD_MODEL_PATH and os.path.exists(LOAD_MODEL_PATH):
    # Se um caminho de modelo foi especificado e o ficheiro existe, CARREGA o modelo.
    print(f"A carregar modelo existente de: {LOAD_MODEL_PATH}")
    model = PPO.load(LOAD_MODEL_PATH, env=env)

    # Reconfigura o logger para que o TensorBoard continue os gráficos do treino anterior
    new_logger = configure(logdir, ["stdout", "csv", "tensorboard"])
    model.set_logger(new_logger)
    print("Modelo carregado. O treino vai continuar a partir deste ponto.")
else:
    # Se não, cria um modelo novo do zero.
    print("Nenhum modelo para carregar encontrado. A criar um novo modelo do zero.")
    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=logdir
    )

# --- Configuração de Callbacks (igual a antes) ---
checkpoint_callback = CheckpointCallback(
    save_freq=20001,
    save_path=models_dir,
    name_prefix="av_driver_model"
)

# --- Iniciar ou Continuar o Treino ---
# Pode aumentar o número de passos para o próximo objetivo de treino
# O total de passos será a soma dos passos anteriores + estes novos.
TIMESTEPS_TO_ADD = 50000

print(f"A treinar por mais {TIMESTEPS_TO_ADD} timesteps...")
try:
    # O `reset_num_timesteps=False` é CRUCIAL para continuar a contagem de passos
    model.learn(
        total_timesteps=TIMESTEPS_TO_ADD,
        reset_num_timesteps=False,
        tb_log_name="PPO",
        callback=checkpoint_callback
    )

    # Calcula o total de passos para o nome do ficheiro
    total_steps = model.num_timesteps
    model.save(f"{models_dir}/final_model_{total_steps}")
    print(f"Treino concluído e modelo final guardado com {total_steps} passos totais.")

except KeyboardInterrupt:
    total_steps = model.num_timesteps
    model.save(f"{models_dir}/manual_interrupt_model_{total_steps}")
    print(f"\nTreino interrompido. Modelo guardado com {total_steps} passos.")
finally:
    env.close()