from gymnasium.utils.env_checker import check_env
from gym_env import WebotsAVEnv

env = WebotsAVEnv()
check_env(env) # .unwrapped se tiveres wrappers, senão só env
print("check_env concluído.")
env.close()