import time
from gym_env import WebotsAVEnv  # Importa a sua classe do ambiente
import numpy as np

# --- Parâmetros do Teste ---
NUM_EPISODES = 5  # Quantos episódios quer correr
MAX_STEPS_PER_EPISODE = 1000  # Limite de passos para evitar loops infinitos


def choose_heuristic_action(observation: np.ndarray) -> np.ndarray:
    """
    Esta é a "inteligência" do nosso agente de teste.
    Ele segue regras simples para decidir se acelera ou trava.

    A observação é: [velocidade, dist_frente, dist_esquerda, dist_direita]
    """
    # Descompactar a observação para facilitar a leitura
    speed_norm, front_dist_norm, left_dist_norm, right_dist_norm = observation

    # Converter distâncias normalizadas de volta para uma estimativa em metros para a lógica
    # (Assumindo que lidar_max_range é 10.0, ajuste se for diferente no seu .wbt)
    LIDAR_MAX_RANGE = 50.0  # AJUSTE este valor para o maxRange do seu Lidar
    front_dist_meters = front_dist_norm * LIDAR_MAX_RANGE

    # --- Lógica de Decisão ---

    # 1. REGRA DE EMERGÊNCIA: Se algo está muito perto, trava a fundo!
    if front_dist_meters < 4.0:
        print("  [Heurística]: PERIGO IMINENTE! A travar a fundo.")
        return np.array([-1.0], dtype=np.float32)  # Ação de travagem máxima

    # 2. REGRA DE PRECAUÇÃO: Se algo está a uma distância de atenção, abranda.
    if front_dist_meters < 8.0:
        print(f"  [Heurística]: Obstáculo a {front_dist_meters:.1f}m. A abrandar.")
        return np.array([-0.5], dtype=np.float32)  # Ação de travagem média

    # 3. REGRA PADRÃO: Se o caminho está livre, acelera suavemente.
    print(f"  [Heurística]: Caminho livre (obstáculo a {front_dist_meters:.1f}m). A acelerar.")
    return np.array([0.3], dtype=np.float32)  # Ação de aceleração suave


def run_test():
    """Função principal que corre o ciclo de teste."""
    env = None
    try:
        # Inicializa o seu ambiente Webots
        # Certifique-se que os parâmetros (DEF names) são os mesmos que os do seu treino
        print("A criar o ambiente WebotsAVEnv...")
        env = WebotsAVEnv()
        print("Ambiente criado com sucesso.")

        for episode in range(NUM_EPISODES):
            print("\n" + "=" * 50)
            print(f"A INICIAR EPISÓDIO DE TESTE {episode + 1}/{NUM_EPISODES}")
            print("=" * 50)

            # Reinicia o ambiente para obter a primeira observação
            obs, info = env.reset()

            terminated = False
            truncated = False
            step_count = 0

            while not terminated and not truncated and step_count < MAX_STEPS_PER_EPISODE:
                print(f"\n--- Passo {step_count} ---")

                # O nosso agente heurístico escolhe a ação com base na observação
                action = choose_heuristic_action(obs)

                # Executa a ação no ambiente
                obs, reward, terminated, truncated, info = env.step(action)

                # Imprime os resultados do passo - É AQUI QUE VOCÊ VÊ SE TUDO FUNCIONA!
                print(f"  Observação: [Vel: {obs[0]:.2f}, D_F: {obs[1]:.2f}, D_E: {obs[2]:.2f}, D_D: {obs[3]:.2f}]")
                print(f"  Ação Tomada: {action[0]:.2f}")
                print(f"  Recompensa Recebida: {reward:.4f}")
                print(f"  Terminou: {terminated}, Truncou: {truncated}")

                step_count += 1

            if terminated:
                print("\nEPISÓDIO TERMINADO: O agente colidiu ou chegou ao fim.")
            elif truncated:
                print("\nEPISÓDIO TRUNCADO: Atingiu o limite de passos.")

            print(f"Fim do episódio {episode + 1} após {step_count} passos.")

    except Exception as e:
        print(f"\nOcorreu um erro durante o teste: {e}")
    finally:
        if env:
            print("\nA fechar o ambiente...")
            env.close()
            print("Ambiente fechado.")


if __name__ == "__main__":
    run_test()