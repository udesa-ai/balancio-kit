# ======================================================================
#  Balancio-Kit (c) 2021 Linar (UdeSA)
#  This code is licensed under MIT license (see LICENSE.txt for details)
# ======================================================================
"""
Script for optimizing RL hyperparameters.
Based on: https://github.com/araffin/rl-baselines-zoo/blob/master/utils/hyperparams_opt.py
"""

# Filter tensorflow version warnings
import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
warnings.simplefilter(action='ignore', category=Warning)
import tensorflow as tf
tf.get_logger().setLevel('INFO')
tf.autograph.set_verbosity(0)
import logging
tf.get_logger().setLevel(logging.ERROR)

import pickle as pkl
import yaml
from typing import Any, Dict
import gym
from balancio_lib.environments import balancioGymEnv
from stable_baselines import A2C
from stable_baselines.common.callbacks import EvalCallback
from stable_baselines.common import set_global_seeds
import optuna
from optuna.pruners import MedianPruner
from optuna.visualization import plot_optimization_history, plot_param_importances


# TODO: Add argument parsing

# Environment
NORMALIZE = True
BACKLASH = True
MEMORY_BUFFER = 1
LoopFreq = 100  # Hz
StepPeriod = (1 / 240) * 1 / 10  # s
actions_per_step = int(round((1 / LoopFreq) / StepPeriod))  # For Microcontroller loop frequency compatibility
SEED = 0

# HyperParameters Tuning
RL_ALGO_NAME = "A2C"                # Algorithm to which hp are to be optimized
N_TRIALS = 200                      # Total number of optimization iterations
N_JOBS = 2                          # Number of parallel runs
N_TIMESTEPS = int(1e5)              # Total simulation steps per trial
N_STARTUP_TRIALS = N_TRIALS // 3    # Number of trials before enabling pruning
N_WARMUP_STEPS = N_TIMESTEPS // 3   # Number of steps before enabling pruning, in each trial.
N_EVALUATIONS = 5                   # Number of evaluations in each trial.
N_EVAL_EPISODES = 2                 # Number of episodes to test the agent in each evaluation.
TIMEOUT = 10                         # Timeout in hours


def sample_a2c_params(trial: optuna.Trial) -> Dict[str, Any]:
    """Sampler for A2C hyperparameters."""
    gamma = trial.suggest_categorical("gamma", [0.9, 0.95, 0.98, 0.99, 0.995, 0.999, 0.9999])
    max_grad_norm = trial.suggest_categorical("max_grad_norm", [0.3, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 2, 5])
    alpha = trial.suggest_categorical("alpha", [0.8, 0.9, 0.92, 0.95, 0.98, 0.99, 1.0])
    n_steps = trial.suggest_categorical("n_steps", [8, 16, 32, 64, 128, 256, 512, 1024, 2048])
    vf_coef = trial.suggest_uniform("vf_coef", 0, 1)
    lr_schedule = "constant"  # trial.suggest_categorical("lr_schedule", ["linear", "constant"])
    learning_rate = trial.suggest_loguniform("learning_rate", 1e-5, 1)
    ent_coef = trial.suggest_loguniform("ent_coef", 0.00000001, 0.1)
    # ortho_init = trial.suggest_categorical("ortho_init", [False, True])
    net_arch = trial.suggest_categorical("net_arch", ["tiny", "small", "medium", "big"])
    # activation_fn = trial.suggest_categorical("activation_fn", ["tanh", "relu"])

    # Display true values
    # trial.set_user_attr("gamma_", gamma)
    # trial.set_user_attr("alpha_", alpha)
    # trial.set_user_attr("n_steps", n_steps)

    if net_arch == "tiny":
        net_arch = [16, 16]
    elif net_arch == "small":
        net_arch = [32, 32]
    elif net_arch == "medium":
        net_arch = [64, 64]
    elif net_arch == "big":
        net_arch = [128, 128]

    # net_arch = [64, 64]
    act_fun = tf.nn.relu  # {"tanh": tf.nn.tanh, "relu": tf.nn.relu}[activation_fn]

    return {
        "n_steps": n_steps,
        "gamma": gamma,
        "alpha": alpha,
        "lr_schedule": lr_schedule,
        "learning_rate": learning_rate,
        "vf_coef": vf_coef,
        "ent_coef": ent_coef,
        "max_grad_norm": max_grad_norm,
        "policy_kwargs": {
            "net_arch": net_arch,
            "act_fun": act_fun,
            # "ortho_init": ortho_init,
        },
    }


class TrialEvalCallback(EvalCallback):
    """Callback used for evaluating and reporting a trial."""

    def __init__(
            self,
            eval_env: gym.Env,
            trial: optuna.Trial,
            n_eval_episodes: int = 5,
            eval_freq: int = 10000,
            deterministic: bool = True,
            verbose: int = 0,
    ):

        super().__init__(
            eval_env=eval_env,
            n_eval_episodes=n_eval_episodes,
            eval_freq=eval_freq,
            deterministic=deterministic,
            verbose=verbose,
        )
        self.trial = trial
        self.eval_idx = 0
        self.is_pruned = False

    def _on_step(self) -> bool:
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            super()._on_step()
            self.eval_idx += 1
            self.trial.report(self.last_mean_reward, self.eval_idx)
            # Prune trial if need
            if self.trial.should_prune():
                self.is_pruned = True
                return False
        return True


def objective(trial: optuna.Trial) -> float:
    env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False, normalize=NORMALIZE,
                                        backlash=BACKLASH, memory_buffer=MEMORY_BUFFER)

    DEFAULT_HYPERPARAMS = {
        "policy": "MlpPolicy",
        "env": env,
    }

    kwargs = DEFAULT_HYPERPARAMS.copy()
    # Sample hyperparameters
    kwargs.update(sample_a2c_params(trial))
    # Create the RL model
    model = A2C(**kwargs)
    # Create env used for evaluation
    eval_env = balancioGymEnv.BalancioGymEnv(action_repeat=actions_per_step, renders=False, normalize=NORMALIZE,
                                             backlash=BACKLASH, memory_buffer=MEMORY_BUFFER)
    # eval_env = DummyVecEnv([lambda: eval_env])
    # eval_env = gym.make(ENV_ID)

    # Create the callback that will periodically evaluate
    # and report the performance
    eval_callback = TrialEvalCallback(
        eval_env,
        trial,
        n_eval_episodes=N_EVAL_EPISODES,
        eval_freq=int(N_TIMESTEPS / N_EVALUATIONS),
        deterministic=True,
    )

    nan_encountered = False
    try:
        model.learn(N_TIMESTEPS, callback=eval_callback)
    except AssertionError as e:
        # Sometimes, random hyperparams can generate NaN
        print(e)
        nan_encountered = True
    finally:
        # Free memory
        model.env.close()
        eval_env.close()

    # Tell the optimizer that the trial failed
    if nan_encountered:
        return float("nan")

    if eval_callback.is_pruned:
        raise optuna.exceptions.TrialPruned()

    return eval_callback.last_mean_reward


if __name__ == "__main__":

    sampler = optuna.samplers.CmaEsSampler()
    # sampler = TPESampler(n_startup_trials=N_STARTUP_TRIALS)
    # Do not prune before 1/3 of the max budget is used
    pruner = MedianPruner(
        n_startup_trials=N_STARTUP_TRIALS, n_warmup_steps=N_WARMUP_STEPS
    )

    study = optuna.create_study(sampler=sampler, pruner=pruner, direction="maximize")

    try:
        study.optimize(objective, n_trials=N_TRIALS, n_jobs=N_JOBS, timeout=int(60 * 60 * TIMEOUT))
    except KeyboardInterrupt:
        pass

    print("Number of finished trials: ", len(study.trials))

    print("Best trial:")
    trial = study.best_trial

    print(f"  Value: {trial.value}")

    print("  Params: ")
    for key, value in trial.params.items():
        print(f"    {key}: {value}")

    # print("  User attrs:")
    # for key, value in trial.user_attrs.items():
    #     print(f"    {key}: {value}")

    # Write report
    study.trials_dataframe().to_csv("../rl_data/hyperparameters/HyperParameters_optuna.csv")

    with open("../rl_data/hyperparameters/HP.pkl", "wb+") as f:
        pkl.dump(study, f)
        f.close()

    with open("../rl_data/hyperparameters/HP.yaml", "w") as f:
        param_dict = {RL_ALGO_NAME: study.best_params}
        yaml.dump(param_dict, f)
        f.close()

    fig1 = plot_optimization_history(study)
    fig1.show()
    fig2 = plot_param_importances(study)
    fig2.show()
