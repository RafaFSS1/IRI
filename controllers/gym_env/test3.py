# test_environment.py (ou adapta o teu test2.py)

from gym_env import WebotsAVEnv  # Garante que o nome do ficheiro está correto
import numpy as np
import time
from gymnasium.utils.env_checker import check_env  # Opcional, mas bom


def run_test(env_instance, num_episodes=5, num_steps_per_episode=200):
    """Roda o ambiente com ações aleatórias para teste."""
    for episode in range(num_episodes):
        print(f"\n--- INÍCIO EPISÓDIO {episode + 1}/{num_episodes} ---")
        obs, info = env_instance.reset()
        print(f"Observação Inicial (primeiros 10 de {len(obs)}): {obs[:10]}")

        episode_reward = 0

        for step_num in range(num_steps_per_episode):
            # Escolhe uma ação: aleatória ou fixa para testar algo específico
            # action = env_instance.action_space.sample() # Ação aleatória
            action = np.array([0.1], dtype=np.float32)  # Ação fixa: acelerar um pouco

            obs, reward, terminated, truncated, info = env_instance.step(action)

            episode_reward += reward

            print(f"Ep {episode + 1}, Step {step_num + 1}:")
            # Imprime apenas algumas features da observação para não poluir muito
            # obs[0] é velocidade normalizada
            # obs[1:1+env_instance.num_processed_lidar_features] são as features do Lidar processado
            # obs[1+env_instance.num_processed_lidar_features : FIM] são as features da câmara (placeholders)

            vel_norm = obs[0]
            idx_fim_lidar_proc = 1 + env_instance.lidar_feature_keys
            lidar_proc_feats = obs[1:idx_fim_lidar_proc]
            all_feats = obs[idx_fim_lidar_proc:]

            print(f"  Obs (VelNorm): {vel_norm:.2f}")
            print(f"  Obs (LidarProc Feats {len(lidar_proc_feats)}): {np.round(lidar_proc_feats, 2)}")
            print(f"  Obs (Cam Feats {len(all_feats)}): {np.round(all_feats, 2)}")  # Serão zeros por agora
            print(f"  Recompensa: {reward:.4f}")

            if info and 'reward_debug' in info and info['reward_debug']:
                print(f"  Debug Recompensa: {' | '.join(info['reward_debug'])}")

            if terminated or truncated:
                print(f"EPISÓDIO TERMINADO/TRUNCADO (Term: {terminated}, Trunc: {truncated})")
                break

        print(f"--- FIM EPISÓDIO {episode + 1}/{num_episodes} | Recompensa Total: {episode_reward:.2f} ---")
        if num_episodes > 1:
            print("A aguardar 5s antes do próximo reset para observares o Webots...")
            time.sleep(5)

    env_instance.close()
    print("\nTeste do ambiente concluído.")


if __name__ == "__main__":
    print("A criar o ambiente WebotsAVEnv...")
    # Certifica-te que passas os nomes DEF corretos se eles forem diferentes dos defaults
    env = WebotsAVEnv(
    )

    # Opcional: Correr o check_env primeiro
    # Lembra-te do aviso de não-determinismo que pode aparecer.
    print("\nA iniciar teste de episódios...")
    run_test(env, num_episodes=1, num_steps_per_episode=1000)  # Ajusta números conforme necessário