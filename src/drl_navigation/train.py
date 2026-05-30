from pathlib import Path

from TD3.TD3 import TD3
from ros_python import ROS_env
from replay_buffer import ReplayBuffer
import torch
import numpy as np
from utils import record_eval_positions
from pretrain_utils import Pretraining


def main(args=None):
    action_dim = 2  # linear.x, angular.z
    max_action = 1
    state_dim = 25
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    nr_eval_episodes = 10
    max_epochs = 100
    epoch = 0
    episodes_per_epoch = 30
    episode = 0
    train_every_n = 2
    training_iterations = 300
    batch_size = 40
    max_steps = 200
    steps = 0
    load_saved_buffer = False
    pretrain = False
    pretraining_iterations = 3
    save_every = 10

    model_dir = Path(__file__).parent / "models" / "TD3"
    model_dir.mkdir(parents=True, exist_ok=True)

    model = TD3(
        state_dim=state_dim,
        action_dim=action_dim,
        max_action=max_action,
        device=device,
        save_every=save_every,
        load_model=False,
        save_directory=model_dir,
        load_directory=model_dir,
    )

    ros = ROS_env()
    eval_scenarios = record_eval_positions(n_eval_scenarios=nr_eval_episodes)

    if load_saved_buffer:
        pretraining = Pretraining(
            file_names=[str(Path(__file__).parent / "assets" / "data.yml")],
            model=model,
            replay_buffer=ReplayBuffer(buffer_size=5e3, random_seed=42),
            reward_function=ros.get_reward,
        )
        replay_buffer = pretraining.load_buffer()
        if pretrain:
            pretraining.train(
                pretraining_iterations=pretraining_iterations,
                replay_buffer=replay_buffer,
                iterations=training_iterations,
                batch_size=batch_size,
            )
    else:
        replay_buffer = ReplayBuffer(buffer_size=5e3, random_seed=42)

    latest_scan, distance, cos, sin, collision, goal, a, reward = ros.step(
        lin_velocity=0.0, ang_velocity=0.0
    )

    try:
        while epoch < max_epochs:
            state, terminal = model.prepare_state(
                latest_scan, distance, cos, sin, collision, goal, a
            )
            action = model.get_action(state, True)
            a_in = [(action[0] + 1), action[1]]  # linear: [0,2] m/s

            latest_scan, distance, cos, sin, collision, goal, a, reward = ros.step(
                lin_velocity=a_in[0], ang_velocity=a_in[1]
            )
            next_state, terminal = model.prepare_state(
                latest_scan, distance, cos, sin, collision, goal, a
            )
            replay_buffer.add(state, action, reward, terminal, next_state)

            if terminal or steps == max_steps:
                reason = "COLLISION" if collision else ("GOAL" if goal else "TIMEOUT")
                print(f"Episode {episode} ended ({reason}) | steps={steps} | reward={reward:.2f}")
                latest_scan, distance, cos, sin, collision, goal, a, reward = ros.reset()
                episode += 1
                if episode % train_every_n == 0:
                    model.train(
                        replay_buffer=replay_buffer,
                        iterations=training_iterations,
                        batch_size=batch_size,
                    )
                steps = 0
            else:
                steps += 1

            if (episode + 1) % episodes_per_epoch == 0:
                episode = 0
                epoch += 1
                model.save(filename="TD3", directory=model_dir)
                print(f"Model saved at epoch {epoch}")
                eval(
                    model=model,
                    env=ros,
                    scenarios=eval_scenarios,
                    epoch=epoch,
                    max_steps=max_steps,
                )

    except KeyboardInterrupt:
        pass
    finally:
        print("\nSaving model before exit...")
        model.save(filename="TD3", directory=model_dir)
        print(f"Model saved to {model_dir}")


def eval(model, env, scenarios, epoch, max_steps):
    print("..............................................")
    print(f"Epoch {epoch}. Evaluating {len(scenarios)} scenarios")
    avg_reward = 0.0
    col = 0
    gl = 0
    for scenario in scenarios:
        count = 0
        latest_scan, distance, cos, sin, collision, goal, a, reward = env.eval(
            scenario=scenario
        )
        while count < max_steps:
            state, terminal = model.prepare_state(
                latest_scan, distance, cos, sin, collision, goal, a
            )
            if terminal:
                break
            action = model.get_action(state, False)
            a_in = [(action[0] + 1), action[1]]
            latest_scan, distance, cos, sin, collision, goal, a, reward = env.step(
                lin_velocity=a_in[0], ang_velocity=a_in[1]
            )
            avg_reward += reward
            count += 1
            col += collision
            gl += goal
    avg_reward /= len(scenarios)
    avg_col = col / len(scenarios)
    avg_goal = gl / len(scenarios)
    print(f"Average Reward: {avg_reward:.3f}")
    print(f"Average Collision rate: {avg_col:.3f}")
    print(f"Average Goal rate: {avg_goal:.3f}")
    print("..............................................")
    model.writer.add_scalar("eval/avg_reward", avg_reward, epoch)
    model.writer.add_scalar("eval/avg_col", avg_col, epoch)
    model.writer.add_scalar("eval/avg_goal", avg_goal, epoch)


if __name__ == "__main__":
    main()
