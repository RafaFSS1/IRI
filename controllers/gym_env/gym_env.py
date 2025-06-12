# av_gym_env.py
from typing import Any, Dict

import gymnasium as gym
from gymnasium import spaces
import numpy as np
# from gymnasium.core import ObsType # ObsType não é usado diretamente aqui, Dict sim

from vehicle import Driver  # Driver é um Supervisor
import math


class WebotsAVEnv(gym.Env):
    metadata = {'render_modes': ['human'], 'render_fps': 30}

    def __init__(self,
                 time_step_param=None,
                 av_expected_def_name="VEICULO_TREINO",  # DEF do teu AV (Agente RL)
                 world_supervisor_def_name="RANDOMIZER",
                 overtake_def_name="VEICULO_ULTRUPASSAR",
                 pedestrian1_def_name="PEDESTRIAN_1",
                 pedestrian2_def_name="PEDESTRIAN_2",
                 passadeira1_def_name="PASSADEIRA_1",
                 passadeira2_def_name="PASSADEIRA_2",
                 ):
        super(WebotsAVEnv, self).__init__()
        print("A inicializar WebotsAVEnv...")

        self.driver = Driver()  # O controlador do AV é um Driver (que é um Supervisor)
        self.time_step = time_step_param if time_step_param is not None else int(self.driver.getBasicTimeStep())

        self.av_node = self.driver.getSelf()
        if self.av_node is None:
            # Se o getFromDef falhar (ex: este script não está no AV_DEF_NAME)
            # tenta obter o nó do robô que este controlador está a controlar diretamente
            self.av_node = self.driver.getFromDef(av_expected_def_name)
            if self.av_node is None:
                raise RuntimeError(
                    f"ERRO CRÍTICO: Nó do AV (esperado DEF: '{av_expected_def_name}' ou via getSelf()) não pôde ser obtido.")
            # Verifica se o DEF do nó obtido via getSelf() corresponde ao esperado, se um nome foi dado
            if av_expected_def_name and self.av_node.getDef() != av_expected_def_name:
                print(
                    f"AVISO: Controlador está no Robot DEF '{self.av_node.getDef()}', mas o nome DEF esperado era '{av_expected_def_name}'. A continuar com '{self.av_node.getDef()}'.")

        self.av_translation_field = self.av_node.getField("translation")
        self.av_rotation_field = self.av_node.getField("rotation")
        self.initial_av_translation = list(self.av_translation_field.getSFVec3f())
        self.initial_av_rotation = list(self.av_rotation_field.getSFRotation())
        print(
            f"AV '{self.av_node.getDef()}' encontrado. Pose Inicial: T={self.initial_av_translation}, R={self.initial_av_rotation}")

        # Obter o nó do supervisor do mundo que corre o randomizer.py
        self.world_supervisor_def_name = world_supervisor_def_name
        self.world_supervisor_node = self.driver.getFromDef(self.world_supervisor_def_name)
        if self.world_supervisor_node is None:
            print(f"AVISO CRÍTICO: Nó supervisor do mundo (DEF: '{self.world_supervisor_def_name}') não encontrado. " +
                  "A randomização de cenários via restartController não funcionará.")
        else:
            print(f"Nó supervisor do mundo '{self.world_supervisor_node.getDef()}' encontrado.")

        self.pedestrian1=self.driver.getFromDef(pedestrian1_def_name)
        self.pedestrian2=self.driver.getFromDef(pedestrian2_def_name)
        self.overtake=self.driver.getFromDef(overtake_def_name)
        self.passadeira1=self.driver.getFromDef(passadeira1_def_name)
        self.passadeira2 = self.driver.getFromDef(passadeira2_def_name)

        # --- Definição do Espaço de Ações ---
        # Ação: um valor contínuo entre -1 (travagem máxima) e 1 (aceleração máxima)
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        print(f"Espaço de Ações: {self.action_space}")
        self.max_vehicle_speed_kmh = 80.0  # AJUSTA conforme necessário
        self.current_target_cruising_speed_kmh = 0.0  # Velocidade alvo atual

        # --- Obtenção e Ativação de Sensores ---
        # self.lidar = self.driver.getLidar("LIDAR") # Se usas getLidar
        self.lidar = self.driver.getDevice("lidar")  # Ou getDevice("NOME_DO_TEU_LIDAR_NO_WBT")
        # Confirma que o nome "LIDAR" corresponde ao campo 'name'
        # do teu nó Lidar no ficheiro .wbt

        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()  # <--- ADICIONA ESTA LINHA!
            # Só depois de enablePointCloud() é que getNumberOfPoints() retorna o valor correto
            self.lidar_num_points = self.lidar.getNumberOfPoints()
            self.lidar_max_range = self.lidar.getMaxRange()
            print(
                f"Lidar '{self.lidar.getName()}' ativo com {self.lidar_num_points} pontos, alcance máx: {self.lidar_max_range:.2f}m.")
        else:
            print("AVISO CRÍTICO: Lidar 'LIDAR' não encontrado! Observações estarão incompletas.")
            self.lidar_num_points = 0
            self.lidar_max_range = 10.0

        ## --- Novas Features para Observação (Exemplos) ---
        self.lidar_feature_keys = ['front_dist', 'left_dist', 'right_dist']
        num_lidar_features = len(self.lidar_feature_keys)

        # --- MUDANÇA AQUI ---
        # Adicionamos 1 feature extra para a distância à passadeira
        num_total_features = (
                1 +  # velocidade
                num_lidar_features +
                1  # <<-- NOVA FEATURE: dist_to_crosswalk_norm
        )

        # Limites: [velocidade, lidar_frente, lidar_esquerda, lidar_direita, dist_passadeira]
        obs_low_bounds = [-0.2] + [0.0] * num_lidar_features + [0.0]  # Limite inferior da nova feature
        obs_high_bounds = [1.2] + [1.0] * num_lidar_features + [1.0]  # Limite superior da nova feature

        self.observation_space = spaces.Box(
            low=np.array(obs_low_bounds, dtype=np.float32),
            high=np.array(obs_high_bounds, dtype=np.float32),
            shape=(num_total_features,),  # A forma agora será (5,)
            dtype=np.float32
        )
        print(f"Espaço de Observações EXPANDIDO (size {num_total_features}) definido.")
        print(f"  Breakdown: 1 (vel) + {num_lidar_features} (lidar) + 1 (dist_passadeira)")
        # ...
        # --- Parâmetros do Episódio ---
        self.current_episode_step = 0
        self.max_episode_steps = 100000000000000000000  # AJUSTA: Número máximo de passos por episódio
        self.target_y_position_finish = 500.0  # AJUSTA: Coordenada Y para considerar fim do percurso

        # --- Estado de Colisão (simplificado) ---
        self.collided_this_step = False
        self.min_collision_lidar_distance = 0.3  # AJUSTA: Distância Lidar para considerar colisão (metros)
        self.num_lidar_points_for_collision = 3  # AJUSTA: Quantos pontos Lidar frontais têm de estar abaixo da distância

        print("Ambiente WebotsAVEnv inicializado.")

        # --- Constantes e Limiares para Recompensas (AJUSTA ESTES VALORES!) ---
        self.SPEED_THRESHOLD_STOPPED_MPS = 0.5  # Velocidade considerada parada
        self.SPEED_THRESHOLD_SLOW_CROSSWALK_MPS = 18 / 3.6  # ~5 m/s ou 18 km/h para passadeiras vazias
        self.SPEED_TARGET_SENSIBLE_KMH = 50.0  # Velocidade de cruzeiro desejada

        self.DISTANCE_APPROACH_SCENARIO = 30.0  # Distância (m) para começar a avaliar cenários à frente
        self.DISTANCE_CRITICAL_PEDESTRIAN = 10.0  # Distância muito perto de um peão
        self.DISTANCE_STOP_BEFORE_OBSTACLE = 3.0  # Distância para parar antes de um obstáculo/peão

        # Para ultrapassagem
        self.OVERTAKE_CAR_SIDE_CLEARANCE_X = 5.0  # Distância lateral para considerar seguro
        self.OVERTAKE_CAR_AHEAD_CLEARANCE_Y = 15.0  # Distância à frente para retomar velocidade
        self.OVERTAKE_CAR_BEHIND_CHECK_Y = 0.0  # Distância atrás para verificar carro a aproximar-se

        # Pesos das Recompensas
        self.REWARD_PROGRESS = 0.01  # Por metro progredido em Y
        self.PENALTY_EXCESSIVE_SPEED = -0.05  # Por km/h acima do limite
        self.PENALTY_COLLISION = -2500.0
        self.REWARD_REACH_FINISH = 100.0

        self.REWARD_STOP_PED_CROSSING = 500.0
        self.PENALTY_NOT_STOP_PED_CROSSING = -700.0 #ilegal, logo penalidade alta
        self.REWARD_SLOW_EMPTY_CROSSING = 150.0
        self.PENALTY_FAST_EMPTY_CROSSING = -200.0
        self.REWARD_STOP_JAYWALKING = 500.0
        self.PENALTY_NOT_STOP_JAYWALKING = -700.0 #ilegal também

        self.last_av_y_pos = self.initial_av_translation[1]

        self.pedestrians = []
        if self.pedestrian1: self.pedestrians.append(self.pedestrian1)
        if self.pedestrian2: self.pedestrians.append(self.pedestrian2)

        # No seu __init__, depois de obter os nós das passadeiras
        # ...
        self.crosswalks = []
        if self.passadeira1: self.crosswalks.append(self.passadeira1)
        if self.passadeira2: self.crosswalks.append(self.passadeira2)

        # Guarda as posições Y e as dimensões das passadeiras
        self.crosswalk_info = []
        for cw_node in self.crosswalks:
            pos = cw_node.getField("translation").getSFVec3f()
            # Assumimos que a passadeira tem uma profundidade (ao longo da estrada)
            # e que a sua posição 'Y' é o centro dela.
            depth = 8.0
            self.crosswalk_info.append({
                'y_center': pos[1],
                'y_start': pos[1] - depth / 2,
                'y_end': pos[1] + depth / 2
            })

        print(
            f"Encontradas {len(self.crosswalk_info)} passadeiras com as seguintes posições Y: {[info['y_center'] for info in self.crosswalk_info]}")

        # ...

        self.crosswalk_depth_along_road = 8.0
        self.crosswalk_width_across_road = 12.0

        self.sensor_derived_info = {}

    def _process_lidar_for_features(self) -> dict:
        """
        NOVA VERSÃO ROBUSTA usando getPointCloud().
        Extrai as distâncias mínimas nas zonas de interesse (frente, esquerda, direita).
        """
        features = {
            'front_dist': 1.0,
            'left_dist': 1.0,
            'right_dist': 1.0
        }

        # Obter a nuvem de pontos diretamente do Webots
        # Esta função já nos dá as coordenadas x,y,z de cada ponto detetado.
        point_cloud = self.lidar.getPointCloud()

        if not point_cloud:
            # Se a lista estiver vazia (nenhum ponto detetado), retorna os valores padrão
            self.sensor_derived_info = {'lidar_front_dist_raw': self.lidar_max_range}
            return features

        min_dist_front = self.lidar_max_range
        min_dist_left = self.lidar_max_range
        min_dist_right = self.lidar_max_range

        # Definir "zonas" de interesse.
        # Lembre-se: X é para a frente, Z é para a esquerda.
        front_zone_width_z = 2.0  # Largura da zona frontal (ao longo do eixo Z)
        side_zone_width_z = 3.0  # Largura de cada zona lateral
        side_zone_lookahead_x = 10.0  # Quão à frente a zona lateral verifica
        side_zone_lookbehind_x = 5.0  # Quão atrás a zona lateral verifica

        for point in point_cloud:
            # point.x -> Distância para a frente (+ é frente, - é trás)
            # point.y -> Distância para cima/baixo (+ é cima)
            # point.z -> Distância para a esquerda/direita (+ é esquerda, - é direita)

            dist_euclidiana = math.sqrt(point.x ** 2 + point.z ** 2)  # Distância real ao ponto

            # Verificar se o ponto cai na zona FRONTAL
            # (à frente do carro e não muito para os lados)
            if point.x > 0 and abs(point.z) < front_zone_width_z / 2:
                if point.x < min_dist_front:
                    min_dist_front = point.x  # Usamos a distância frontal (x)

            # Verificar se o ponto cai na zona ESQUERDA (Z positivo)
            if -side_zone_lookbehind_x < point.x < side_zone_lookahead_x and \
                    front_zone_width_z / 2 < point.z < front_zone_width_z / 2 + side_zone_width_z:
                if dist_euclidiana < min_dist_left:
                    min_dist_left = dist_euclidiana

            # Verificar se o ponto cai na zona DIREITA (Z negativo)
            if -side_zone_lookbehind_x < point.x < side_zone_lookahead_x and \
                    -front_zone_width_z / 2 - side_zone_width_z < point.z < -front_zone_width_z / 2:
                if dist_euclidiana < min_dist_right:
                    min_dist_right = dist_euclidiana

        # Guardar o valor bruto mais importante para a função de recompensa
        self.sensor_derived_info['lidar_front_dist_raw'] = min_dist_front

        # Normalizar as distâncias (0.0 = perto, 1.0 = longe)
        features['front_dist'] = np.clip(min_dist_front / self.lidar_max_range, 0.0, 1.0)
        features['left_dist'] = np.clip(min_dist_left / self.lidar_max_range, 0.0, 1.0)
        features['right_dist'] = np.clip(min_dist_right / self.lidar_max_range, 0.0, 1.0)

        return features

    def _get_observation(self) -> np.ndarray:
        # 1. Velocidade Atual (normalizada) - sem alterações
        current_speed_kmh = self.driver.getCurrentSpeed()
        max_speed_mps = self.max_vehicle_speed_kmh / 3.6
        normalized_speed = (current_speed_kmh / 3.6) / max_speed_mps if max_speed_mps > 0 else 0.0
        normalized_speed = np.clip(normalized_speed, self.observation_space.low[0], self.observation_space.high[0])

        # 2. Obter e processar features do Lidar - sem alterações
        processed_lidar_features_dict = self._process_lidar_for_features()
        processed_lidar_features_list = [processed_lidar_features_dict[key] for key in self.lidar_feature_keys]

        # --- MUDANÇA AQUI ---
        # 3. Obter e normalizar a distância à passadeira (o nosso "sensor virtual")
        dist_raw, _ = self._get_distance_to_next_crosswalk()

        # Normalizar a distância: 1.0 = muito perto, 0.0 = muito longe ou nenhuma à frente
        # Usamos DISTANCE_APPROACH_SCENARIO como o alcance máximo do nosso "sensor"
        if math.isinf(dist_raw):
            dist_crosswalk_norm = 0.0
        else:
            # Normalização inversa: quanto mais perto, maior o valor
            dist_crosswalk_norm = np.clip(1.0 - (dist_raw / self.DISTANCE_APPROACH_SCENARIO), 0.0, 1.0)

        # 4. Combina todas as observações
        observation_list = [normalized_speed] + processed_lidar_features_list + [dist_crosswalk_norm]
        observation = np.array(observation_list, dtype=np.float32)
        print(observation)

        if observation.shape != self.observation_space.shape:
             raise ValueError(f"Incompatibilidade no shape da observação. Esperado: {self.observation_space.shape}, Obtido: {observation.shape}")

        return observation


    def _apply_action(self, action: np.ndarray):
        action_value = float(action[0])  # Ação é um array com um elemento

        # Interpretar a ação:
        # action_value > 0: Acelerar
        # action_value < 0: Travar
        # action_value = 0: Manter (nem acelerar, nem travar ativamente)

        if action_value > 0:  # Acelerar
            self.driver.setBrakeIntensity(0.0)  # Soltar o travão
            # Aumentar a velocidade alvo de cruzeiro com base no esforço da ação
            # Ex: action_value = 1 significa 10% da vel max para incremento
            speed_increment_kmh = action_value * (
                        self.max_vehicle_speed_kmh * 0.15)  # AJUSTA este fator de sensibilidade
            self.current_target_cruising_speed_kmh += speed_increment_kmh
            # Limitar a velocidade alvo à máxima permitida e não permitir negativa
            self.current_target_cruising_speed_kmh = np.clip(self.current_target_cruising_speed_kmh, 0,
                                                             self.max_vehicle_speed_kmh)
            self.driver.setCruisingSpeed(self.current_target_cruising_speed_kmh)
        elif action_value < 0:  # Travar
            # A intensidade da travagem é proporcional ao valor negativo da ação
            brake_intensity = -action_value  # De 0 (sem travagem) a 1 (travagem máxima)
            brake_intensity = np.clip(brake_intensity, 0.0, 1.0)
            self.driver.setBrakeIntensity(brake_intensity)
            # Se estiver a travar com força, pode ser útil reduzir a velocidade alvo para ajudar a parar
            if brake_intensity > 0.7:  # AJUSTA este limiar
                self.current_target_cruising_speed_kmh = 0
                self.driver.setCruisingSpeed(0)  # Comanda o carro para tentar parar
        else:  # Sem esforço de aceleração/travagem explícito (action_value == 0)
            self.driver.setBrakeIntensity(0.0)  # Certifica-te que o travão não está aplicado
            # Mantém a velocidade alvo de cruzeiro atual (o carro tentará mantê-la)
            self.driver.setCruisingSpeed(self.current_target_cruising_speed_kmh)

    def _check_collision_lidar(self) -> bool:
        # A nossa função _get_observation já calcula a distância frontal e guarda em sensor_derived_info
        # Já não precisamos de recalcular aqui.

        front_dist = self.sensor_derived_info.get('lidar_front_dist_raw', self.lidar_max_range)

        if front_dist < self.min_collision_lidar_distance:
            print(
                f"COLISÃO DETETADA: Obstáculo frontal a {front_dist:.2f}m (limite: {self.min_collision_lidar_distance}m)")
            return True

        return False

    def _get_distance_to_next_crosswalk(self) -> (float, bool):
        """
        Calcula a distância até à próxima passadeira à frente do veículo.
        Usa a posição dos nós (visão de supervisor), não sensores.
        Retorna:
            - dist_y (float): Distância em metros até o início da próxima passadeira. `float('inf')` se nenhuma estiver à frente.
            - is_on_crosswalk (bool): True se o centro do carro estiver sobre uma passadeira.
        """
        av_pos_y = self.av_translation_field.getSFVec3f()[1]
        self.crosswalk_info=[]

        for cw_node in self.crosswalks:
            pos = cw_node.getField("translation").getSFVec3f()
            # Assumimos que a passadeira tem uma profundidade (ao longo da estrada)
            # e que a sua posição 'Y' é o centro dela.
            depth = 8.0
            self.crosswalk_info.append({
                'y_center': pos[1],
                'y_start': pos[1] - depth / 2,
                'y_end': pos[1] + depth / 2
            })

        nearest_dist = float('inf')
        is_on = False

        for cw in self.crosswalk_info:
            # Verifica se o carro está sobre esta passadeira
            if cw['y_start'] <= av_pos_y <= cw['y_end']:
                is_on = True
                # Se estamos em cima, a distância para a próxima é irrelevante para esta lógica
                # Podemos retornar 0 ou continuar a procurar a próxima
                nearest_dist = 0
                break  # Encontramos o estado atual, podemos parar

            # Verifica se a passadeira está à frente do carro
            elif cw['y_start'] > av_pos_y:
                dist = cw['y_start'] - av_pos_y
                if dist < nearest_dist:
                    nearest_dist = dist

        return nearest_dist, is_on

    def _calculate_reward(self) -> float:
        reward = 0.0
        debug_prints = []
        current_speed_kmh = self.driver.getCurrentSpeed()
        current_speed_mps = current_speed_kmh / 3.6
        av_pos_vec3 = self.av_translation_field.getSFVec3f()
        av_pos_y = av_pos_vec3[1]

        # --- Recompensa por Progresso ---
        progress_y = av_pos_y - self.last_av_y_pos
        reward += progress_y * self.REWARD_PROGRESS
        self.last_av_y_pos = av_pos_y
        if progress_y > 0.01: debug_prints.append(f"Prog: {reward:.2f}")

        # --- Lógica de Recompensa Baseada no Lidar ---
        # Obter distâncias brutas que foram guardadas em self.sensor_derived_info
        front_dist = self.sensor_derived_info.get('lidar_front_dist_raw', self.lidar_max_range)

        # 1. Penalidade por Proximidade Perigosa (Safety Driver)
        # Penalidade aumenta exponencialmente quanto mais perto e mais rápido.
        SAFE_DISTANCE_MPS = self.DISTANCE_STOP_BEFORE_OBSTACLE  # 3m
        if front_dist < SAFE_DISTANCE_MPS * 2:  # Começa a penalizar abaixo de 6m
            # O fator (current_speed_mps + 1) torna a penalidade muito pior a alta velocidade
            proximity_penalty = (1 - (front_dist / (SAFE_DISTANCE_MPS * 2))) * (current_speed_mps + 1)
            reward -= proximity_penalty * 10  # Ajusta o peso desta penalidade
            debug_prints.append(
                f"PENAL_Prox: {-proximity_penalty * 10:.2f} (d:{front_dist:.1f}m, s:{current_speed_kmh:.1f}km/h)")

        # 2. Recompensa por Eficiência (Manter Velocidade Alvo)
        # Só dá esta recompensa se não houver perigo iminente à frente.
        if front_dist > 20.0:  # Se o caminho estiver livre
            target_speed_mps = self.SPEED_TARGET_SENSIBLE_KMH / 3.6
            speed_diff = abs(current_speed_mps - target_speed_mps)
            # Recompensa baseada na gaussiana: máxima na velocidade alvo, decai suavemente.
            speed_reward = math.exp(-0.1 * (speed_diff ** 2))
            reward += speed_reward * 0.05  # Ajusta o peso desta recompensa
            debug_prints.append(f"REW_Speed: {speed_reward * 0.05:.2f}")

            if current_speed_mps < target_speed_mps * 0.5 and current_speed_kmh > 1.0:  # Abaixo de 50% da vel. alvo
                reward -= 1.5
                debug_prints.append("PENAL_Lento: -1.5")

        # 3. Penalidade por estar parado sem razão
        if current_speed_mps < self.SPEED_THRESHOLD_STOPPED_MPS and front_dist > 15.0:
            reward -= 1
            debug_prints.append(f"PENAL_Stopped: -1.0")

        dist_to_crosswalk, is_on_crosswalk = self._get_distance_to_next_crosswalk()
            # Obter distância ao obstáculo frontal a partir do Lidar (que já calculámos)
        front_dist_lidar = self.sensor_derived_info.get('lidar_front_dist_raw', self.lidar_max_range)

            # Cenário 1: Aproximar-se de uma passadeira
        if dist_to_crosswalk < self.DISTANCE_APPROACH_SCENARIO:

                # Sub-cenário 1.1: Passadeira está VAZIA (Lidar não vê nada perto)
            if front_dist_lidar > dist_to_crosswalk:  # O obstáculo mais próximo está mais longe que a passadeira
                debug_prints.append(f"SCEN: Passadeira Vazia à frente ({dist_to_crosswalk:.1f}m)")
                target_slow_mps = self.SPEED_THRESHOLD_SLOW_CROSSWALK_MPS

                    # Recompensa por abrandar para a velocidade segura
                if current_speed_mps - target_slow_mps < 2.0:  # Tolerância de ~7 km/h
                    reward += self.REWARD_SLOW_EMPTY_CROSSING
                    debug_prints.append(f"REW_SlowCross: {self.REWARD_SLOW_EMPTY_CROSSING:.2f}")
                else:
                    reward += self.PENALTY_FAST_EMPTY_CROSSING
                    debug_prints.append(f"PENAL_FastCross: {self.PENALTY_FAST_EMPTY_CROSSING:.2f}")

                # Sub-cenário 1.2: Passadeira está OCUPADA (Lidar deteta um obstáculo nela)
                # Consideramos "ocupada" se o Lidar vê algo a uma distância semelhante à da passadeira
            elif abs(front_dist_lidar - dist_to_crosswalk) < 5.0:  # Obstáculo a +-5m da passadeira
                debug_prints.append(f"SCEN: Peão/Obst. na Passadeira ({dist_to_crosswalk:.1f}m)")

                    # Recompensa por parar ou estar quase parado
                if current_speed_mps < self.SPEED_THRESHOLD_STOPPED_MPS:
                    reward += self.REWARD_STOP_PED_CROSSING
                    debug_prints.append(f"REW_StopPed: {self.REWARD_STOP_PED_CROSSING:.2f}")
                else:
                        # Penalidade forte por não parar
                    reward += self.PENALTY_NOT_STOP_PED_CROSSING
                    debug_prints.append(f"PENAL_NotStopPed: {self.PENALTY_NOT_STOP_PED_CROSSING:.2f}")

        elif front_dist_lidar < self.DISTANCE_CRITICAL_PEDESTRIAN:
            debug_prints.append(f"SCEN: Peão Jaywalking ou carro em cima? (d:{front_dist_lidar:.1f}m)")
            if current_speed_mps < self.SPEED_THRESHOLD_STOPPED_MPS:
                reward += self.REWARD_STOP_JAYWALKING
                debug_prints.append(f"REW_StopJaywalk: {self.REWARD_STOP_JAYWALKING:.2f}")
            else:
                proximity_factor = 1 - (front_dist_lidar / self.DISTANCE_APPROACH_SCENARIO)
                penalty = self.PENALTY_NOT_STOP_JAYWALKING * proximity_factor
                reward += penalty
                debug_prints.append(f"PENAL_NotStopJaywalk: {penalty:.2f}")

        if self.collided_this_step:
            reward += self.PENALTY_COLLISION
            debug_prints.append(f"PENAL:COLISAO! {self.PENALTY_COLLISION}")

        if av_pos_y >= self.target_y_position_finish - 5 and not self.collided_this_step:
            reward += self.REWARD_REACH_FINISH
            debug_prints.append(f"REW:FIM! {self.REWARD_REACH_FINISH}")

        if debug_prints:
            print(f"Step {self.current_episode_step} | Rew: {reward:.3f} | {' | '.join(debug_prints)}")

        return reward

    def _is_terminated(self) -> bool:
        # Condição de terminação 1: Colisão
        self.collided_this_step = self._check_collision_lidar()  # Atualiza a flag de colisão
        if self.collided_this_step:
            print("EPISÓDIO TERMINADO: Colisão.")
            return True

        # Condição de terminação 2: Saiu da estrada (exemplo simples baseado na posição X)
        current_pos = self.av_translation_field.getSFVec3f()
        max_x_deviation = 10.0  # AJUSTA: Quão longe do centro da estrada (X=0) é considerado "fora"
        if abs(current_pos[0]) > max_x_deviation:
            print(f"EPISÓDIO TERMINADO: Saiu da estrada (X={current_pos[0]:.2f}).")
            self.collided_this_step = True  # Considera isto uma forma de colisão/falha grave
            return True

        # Condição de terminação 3: Chegou ao fim do percurso
        if current_pos[1] >= self.target_y_position_finish:
            print(f"EPISÓDIO TERMINADO (SUCESSO): Chegou ao fim do percurso (Y={current_pos[1]:.2f}).")
            return True

        return False

    def _is_truncated(self) -> bool:
        # Condição de truncamento: Número máximo de passos atingido
        if self.current_episode_step >= self.max_episode_steps:
            print(f"EPISÓDIO TRUNCADO: Número máximo de passos ({self.max_episode_steps}) atingido.")
            return True
        return False

    def step(self, action: np.ndarray) -> tuple[np.ndarray, float, bool, bool, Dict[str, Any]]:
        self.current_episode_step += 1

        self._apply_action(action)

        # Simular um passo no Webots
        if self.driver.step() == -1:
            # Simulação terminou externamente (ex: Webots fechado)
            print("SIMULAÇÃO WEBOTS TERMINADA EXTERNAMENTE.")
            empty_obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return empty_obs, 0.0, True, True, {}  # Terminated, Truncated, Info

        observation = self._get_observation()
        terminated = self._is_terminated()  # Verifica colisão, fim do percurso, etc.
        reward = self._calculate_reward()  # Calcula recompensa APÓS verificar colisão/fim
        truncated = self._is_truncated()

        info = {}  # Para informação de depuração adicional

        # print(f"Step {self.current_episode_step}: Act: {action[0]:.2f}, Rew: {reward:.2f}, Term: {terminated}, Trunc: {truncated}, Speed: {(self.driver.getCurrentSpeed()/3.6):.1f} m/s, Target: {self.current_target_cruising_speed_kmh:.1f} km/h")

        return observation, reward, terminated, truncated, info

    def reset(self, seed=None, options=None) -> tuple[np.ndarray, Dict[str, Any]]:
        super().reset(seed=seed)  # Importante para a gestão da seed no Gymnasium
        print("GYM_ENV: Episódio a reiniciar...")
        self.current_episode_step = 0
        self.collided_this_step = False
        self.current_target_cruising_speed_kmh = 0.0

        # 1. Parar qualquer movimento anterior do AV e resetar física da simulação
        self.driver.setBrakeIntensity(1.0)
        # Dar alguns passos para o travão atuar e a simulação estabilizar antes do reset de física
        for _ in range(5):
            if self.driver.step() == -1: break  # Sair se a simulação terminar

        self.driver.simulationResetPhysics()  # Reseta a física de toda a simulação

        # Aguardar um pouco para a física estabilizar após o reset global
        for _ in range(10):
            if self.driver.step() == -1: break

        # 2. Reposicionar o AV para a sua pose inicial
        if self.av_translation_field and self.av_rotation_field:
            self.av_translation_field.setSFVec3f(self.initial_av_translation)
            self.av_rotation_field.setSFRotation(self.initial_av_rotation)
            if self.av_node: self.av_node.resetPhysics()  # Reseta a física só do AV após teletransporte

            # Garantir que o estado do AV é consistente após reposicionamento
            for _ in range(5):  # Alguns passos para aplicar mudanças
                if self.driver.step() == -1: break
            self.driver.setCruisingSpeed(0.0)
            self.driver.setBrakeIntensity(0.0)  # Liberta o travão

            print(f"  GYM_ENV: AV '{self.av_node.getDef()}' reposicionado para pose inicial. Velocidade alvo: 0 km/h.")
        else:
            print("  GYM_ENV: ERRO no reset: campos de translação/rotação do AV não encontrados.")
            return np.zeros(self.observation_space.shape, dtype=np.float32), {}

        # 3. Reiniciar o controlador do supervisor do mundo para reconfigurar o cenário e os controladores dos obejtos
        if self.world_supervisor_node:
            print(
                f"  GYM_ENV: A reiniciar o controlador do nó supervisor do mundo ('{self.world_supervisor_node.getDef()}')...")
            self.world_supervisor_node.restartController()
            self.overtake.restartController()
            self.pedestrian1.restartController()
            self.pedestrian2.restartController()
            # Dar tempo ao randomizer.py (reiniciado) para executar setup_world_scenarios()
            print("  GYM_ENV: A aguardar que o supervisor do mundo reconfigure o cenário...")
            for _ in range(30):  # AJUSTA este número de passos de espera conforme necessário
                if self.driver.step() == -1: break
            print("  GYM_ENV: Espera pela reconfiguração do cenário concluída.")
        else:
            print(
                f"  GYM_ENV: AVISO no reset: Nó do supervisor do mundo (DEF: '{self.world_supervisor_def_name}') não encontrado. Cenário não reconfigurado.")

        # Reativa os sensores (pode ser redundante se já ativos, mas garante o estado)
        if self.lidar: self.lidar.enable(self.time_step)
        # Dar um passo para garantir que os sensores têm leituras atualizadas após o reset
        if self.driver.step() == -1:
            return np.zeros(self.observation_space.shape, dtype=np.float32), {}

        # 4. Obter a observação inicial após o reset completo
        observation = self._get_observation()
        info = {}
        print("GYM_ENV: Reset do episódio completo. A retornar observação inicial.")
        return observation, info

    def render(self, mode='human'):
        # Webots trata da renderização visual. Este método é mais para compatibilidade Gym.
        pass

    def close(self):
        # Limpeza, se necessária (ex: parar explicitamente o robô).
        # Webots geralmente trata do fecho da simulação.
        print("Ambiente WebotsAVEnv a fechar.")
        self.driver.setCruisingSpeed(0.0)
        self.driver.setBrakeIntensity(1.0)
        # self.driver.simulationQuit(0) # Considera se queres que o Python feche a simulação
        pass