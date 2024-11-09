#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile
import numpy as np
import time
import torch
import torch.nn as nn
from torch.distributions import Categorical
import os
from datetime import datetime
import threading
import random
import math

# Device configuration
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Constants
MAX_LIN_VEL = 1.5
LIN_VEL_STEP_SIZE = 0.1

class Memory:
    def __init__(self):
        self.actions = []
        self.states = []
        self.logprobs = []
        self.rewards = []
        self.is_terminals = []
    
    def clear_memory(self):
        del self.actions[:]
        del self.states[:]
        del self.logprobs[:]
        del self.rewards[:]
        del self.is_terminals[:]

class ActorCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(ActorCritic, self).__init__()
        
        # Actor network
        self.actor = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, action_dim),
            nn.Softmax(dim=-1)
        )
        
        # Critic network
        self.critic = nn.Sequential(
            nn.Linear(state_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, 1)
        )
        
    def forward(self):
        raise NotImplementedError
        
    def act(self, state, memory):
        state = torch.FloatTensor(state).to(device)
        action_probs = self.actor(state)
        dist = Categorical(action_probs)
        action = dist.sample()
        
        memory.states.append(state)
        memory.actions.append(action)
        memory.logprobs.append(dist.log_prob(action))
        
        return action.item()
    
    def evaluate(self, state, action):
        action_probs = self.actor(state)
        dist = Categorical(action_probs)
        
        action_logprobs = dist.log_prob(action)
        dist_entropy = dist.entropy()
        state_value = self.critic(state)
        
        return action_logprobs, torch.squeeze(state_value), dist_entropy

class PPO:
    def __init__(self, state_dim, action_dim, lr_actor=0.0003, lr_critic=0.001, gamma=0.99, K_epochs=80, eps_clip=0.2):
        self.gamma = gamma
        self.eps_clip = eps_clip
        self.K_epochs = K_epochs
        
        self.policy = ActorCritic(state_dim, action_dim).to(device)
        self.optimizer = torch.optim.Adam([
            {'params': self.policy.actor.parameters(), 'lr': lr_actor},
            {'params': self.policy.critic.parameters(), 'lr': lr_critic}
        ])
        
        self.policy_old = ActorCritic(state_dim, action_dim).to(device)
        self.policy_old.load_state_dict(self.policy.state_dict())
        
        self.MseLoss = nn.MSELoss()
        self.memory = Memory()

    def select_action(self, state):
        with torch.no_grad():
            return self.policy_old.act(state, self.memory)

    def update(self):
        rewards = []
        discounted_reward = 0
        for reward, is_terminal in zip(reversed(self.memory.rewards), reversed(self.memory.is_terminals)):
            if is_terminal:
                discounted_reward = 0
            discounted_reward = reward + (self.gamma * discounted_reward)
            rewards.insert(0, discounted_reward)
        
        rewards = torch.tensor(rewards, dtype=torch.float32).to(device)
        rewards = (rewards - rewards.mean()) / (rewards.std() + 1e-5)
        
        old_states = torch.stack(self.memory.states).detach()
        old_actions = torch.stack(self.memory.actions).detach()
        old_logprobs = torch.stack(self.memory.logprobs).detach()

        for _ in range(self.K_epochs):
            logprobs, state_values, dist_entropy = self.policy.evaluate(old_states, old_actions)
            
            ratios = torch.exp(logprobs - old_logprobs.detach())
            advantages = rewards - state_values.detach()
            surr1 = ratios * advantages
            surr2 = torch.clamp(ratios, 1-self.eps_clip, 1+self.eps_clip) * advantages
            
            loss = -torch.min(surr1, surr2) + 0.5*self.MseLoss(state_values, rewards) - 0.01*dist_entropy
            
            self.optimizer.zero_grad()
            loss.mean().backward()
            self.optimizer.step()
            
        self.policy_old.load_state_dict(self.policy.state_dict())
        self.memory.clear_memory()

    def save(self, checkpoint_path):
        torch.save(self.policy_old.state_dict(), checkpoint_path)
   
    def load(self, checkpoint_path):
        self.policy_old.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))
        self.policy.load_state_dict(torch.load(checkpoint_path, map_location=lambda storage, loc: storage))

class GazeboEnv(Node):
    def __init__(self, node_name='ppo_training_node'):
        super().__init__(node_name)
        
        # Publishers
        self.publisher_robot1 = self.create_publisher(Twist, '/Robot1/cmd_vel', 10)
        self.publisher_robot2 = self.create_publisher(Twist, '/Robot2/cmd_vel', 10)
        
        # Subscribers
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10)
            
        # Service clients
        self.set_state = self.create_client(SetModelState, '/gazebo/set_model_state')
        self.reset_world = self.create_client(Empty, '/reset_world')
        self.pause_physics = self.create_client(Empty, '/pause_physics')
        self.unpause_physics = self.create_client(Empty, '/unpause_physics')
        
        # Control variables
        self.target_linear_x1 = 0.0
        self.target_linear_y1 = 0.0
        self.control_linear_x1 = 0.0
        self.control_linear_y1 = 0.0
        
        self.target_linear_x2 = 0.0
        self.target_linear_y2 = 0.0
        self.control_linear_x2 = 0.0
        self.control_linear_y2 = 0.0

        # Previous velocities for reward calculation
        self.prev_linear_x1 = 0.0
        self.prev_linear_y1 = 0.0
        self.prev_linear_x2 = 0.0
        self.prev_linear_y2 = 0.0
        
        # State variables
        self.robot1_pos = np.zeros(2)
        self.robot2_pos = np.zeros(2)
        self.ball_pos = np.zeros(3)
        self.prev_ball_pos = np.zeros(3)
        self.ball_velocity = np.zeros(3)
        
        # Game state
        self.done = False
        self.t = 0
        self.ball_hit_count = 0
        self.robot1_life = 5
        self.robot2_life = 5
        
        self.get_logger().info('Environment initialized')

    def model_states_callback(self, msg):
        try:
            robot1_idx = msg.name.index('Robot1')
            robot2_idx = msg.name.index('Robot2')
            ball_idx = msg.name.index('unit_sphere')
            
            # Store previous ball position
            self.prev_ball_pos = self.ball_pos.copy()
            
            # Update positions
            self.robot1_pos[0] = msg.pose[robot1_idx].position.x
            self.robot1_pos[1] = msg.pose[robot1_idx].position.y
            
            self.robot2_pos[0] = msg.pose[robot2_idx].position.x
            self.robot2_pos[1] = msg.pose[robot2_idx].position.y
            
            self.ball_pos[0] = msg.pose[ball_idx].position.x
            self.ball_pos[1] = msg.pose[ball_idx].position.y
            self.ball_pos[2] = msg.pose[ball_idx].position.z
            
            # Calculate ball velocity
            self.ball_velocity = (self.ball_pos - self.prev_ball_pos) / 0.05
            
        except ValueError as e:
            self.get_logger().warn(f'Model state callback error: {str(e)}')

    def reset(self):
        """Reset the environment state"""
        # Reset all control variables
        self.target_linear_x1 = 0.0
        self.target_linear_y1 = 0.0
        self.control_linear_x1 = 0.0
        self.control_linear_y1 = 0.0
        
        self.target_linear_x2 = 0.0
        self.target_linear_y2 = 0.0
        self.control_linear_x2 = 0.0
        self.control_linear_y2 = 0.0
        
        # Reset game state
        self.t = 0
        self.done = False
        self.ball_hit_count = 0

        if self.done:
            self.robot1_life = 5
            self.robot2_life = 5

        # Reset robot positions using SetModelState service
        robot1_state = ModelState()
        robot1_state.model_name = 'Robot1'
        robot1_state.pose.position.x = -0.8
        robot1_state.pose.position.y = 0.2
        robot1_state.pose.position.z = 0.6

        robot2_state = ModelState()
        robot2_state.model_name = 'Robot2'
        robot2_state.pose.position.x = 0.8
        robot2_state.pose.position.y = 0.2
        robot2_state.pose.position.z = 0.6

        # Reset ball with random velocity
        ball_state = ModelState()
        ball_state.model_name = 'unit_sphere'
        ball_state.pose.position.x = 0.0
        ball_state.pose.position.y = 0.0
        ball_state.pose.position.z = 0.6
        
        # Set random initial velocity for the ball
        ball_state.twist.linear.x = -1 * (0.3 + 0.6 * random.random())
        ball_state.twist.linear.y = 0.6 * (random.random() - 0.5)
        ball_state.twist.linear.z = 0.0

        # Apply resets with retry logic
        MAX_RETRIES = 3
        for retry in range(MAX_RETRIES):
            try:
                # Reset world first
                self.reset_world.call_async(Empty.Request())
                time.sleep(0.1)  # Wait for reset to take effect

                # Reset individual elements
                request = SetModelState.Request()
                
                # Reset Robot1
                request.model_state = robot1_state
                self.set_state.call_async(request)
                
                # Reset Robot2
                request.model_state = robot2_state
                self.set_state.call_async(request)
                
                # Reset Ball
                request.model_state = ball_state
                self.set_state.call_async(request)
                
                break
            except Exception as e:
                if retry == MAX_RETRIES - 1:
                    self.get_logger().error(f'Failed to reset after {MAX_RETRIES} attempts: {str(e)}')
                    raise
                time.sleep(0.1)

        # Small delay to allow physics to settle
        time.sleep(0.1)
        
        return self.get_states()

    def calculate_rewards(self):
        reward1 = 0
        reward2 = 0
        
        # 1. Base positioning rewards/penalties
        if self.robot1_pos[0] > 0:  # Red robot crossing center
            reward1 -= 2.0
        if self.robot2_pos[0] < 0:  # Blue robot crossing center
            reward2 -= 2.0
        
        # 2. Ball proximity reward when in correct position
        robot1_ball_dist = np.linalg.norm(self.ball_pos[:2] - self.robot1_pos)
        robot2_ball_dist = np.linalg.norm(self.ball_pos[:2] - self.robot2_pos)
        
        # Only reward getting closer to ball when it's in their half
        if self.ball_pos[0] < 0:  # Ball in red's half
            if robot1_ball_dist < 2.0:  # Close enough to potentially hit
                reward1 += (2.0 - robot1_ball_dist) * 2.0
        else:  # Ball in blue's half
            if robot2_ball_dist < 2.0:  # Close enough to potentially hit
                reward2 += (2.0 - robot2_ball_dist) * 2.0
        
        # 3. Ball hitting rewards with direction consideration
        ball_speed = np.linalg.norm(self.ball_velocity)
        if ball_speed > 0.5:  # Ball is moving significantly
            # Reward hitting ball towards opponent's side
            if robot1_ball_dist < 1.0 and self.ball_pos[0] < 0:
                if self.ball_velocity[0] > 0:  # Ball moving towards blue's side
                    reward1 += 10.0 * ball_speed
                    self.ball_hit_count += 1
            if robot2_ball_dist < 1.0 and self.ball_pos[0] > 0:
                if self.ball_velocity[0] < 0:  # Ball moving towards red's side
                    reward2 += 10.0 * ball_speed
                    self.ball_hit_count += 1
        
        # 4. Penalize excessive movement (continued)
        velocity_change1 = abs(self.control_linear_x1 - self.prev_linear_x1) + \
                        abs(self.control_linear_y1 - self.prev_linear_y1)
        velocity_change2 = abs(self.control_linear_x2 - self.prev_linear_x2) + \
                        abs(self.control_linear_y2 - self.prev_linear_y2)
        
        reward1 -= 0.5 * velocity_change1
        reward2 -= 0.5 * velocity_change2
        
        # 5. Goal scoring/life system
        if self.ball_pos[0] < -12.3:  # Ball in red goal
            self.robot1_life -= 1
            reward1 -= 20.0
            reward2 += 20.0
            self.done = self.robot1_life <= 0
            self.get_logger().info('Robot 1 lost a point! Lives: %d' % self.robot1_life)
        elif self.ball_pos[0] > 12.3:  # Ball in blue goal
            self.robot2_life -= 1
            reward1 += 20.0
            reward2 -= 20.0
            self.done = self.robot2_life <= 0
            self.get_logger().info('Robot 2 lost a point! Lives: %d' % self.robot2_life)
        
        # 6. Timeout conditions
        if self.t >= 1000:
            if self.ball_hit_count == 0:
                reward1 -= 15.0
                reward2 -= 15.0
            self.done = True
            self.get_logger().info('Episode timeout! Ball hits: %d' % self.ball_hit_count)
        
        # Store previous values
        self.prev_linear_x1 = self.control_linear_x1
        self.prev_linear_y1 = self.control_linear_y1
        self.prev_linear_x2 = self.control_linear_x2
        self.prev_linear_y2 = self.control_linear_y2
        
        return reward1, reward2

    def step(self, action1, action2):
        self.t += 1
        
        # Process Robot1 action
        if action1 == 0:  # forward
            self.target_linear_x1 = MAX_LIN_VEL
            self.target_linear_y1 = 0.0
        elif action1 == 1:  # backward
            self.target_linear_x1 = -MAX_LIN_VEL
            self.target_linear_y1 = 0.0
        elif action1 == 2:  # left
            self.target_linear_x1 = 0.0
            self.target_linear_y1 = MAX_LIN_VEL
        elif action1 == 3:  # right
            self.target_linear_x1 = 0.0
            self.target_linear_y1 = -MAX_LIN_VEL
        else:  # stop
            self.target_linear_x1 = 0.0
            self.target_linear_y1 = 0.0

        # Process Robot2 action
        if action2 == 0:  # forward
            self.target_linear_x2 = MAX_LIN_VEL
            self.target_linear_y2 = 0.0
        elif action2 == 1:  # backward
            self.target_linear_x2 = -MAX_LIN_VEL
            self.target_linear_y2 = 0.0
        elif action2 == 2:  # left
            self.target_linear_x2 = 0.0
            self.target_linear_y2 = MAX_LIN_VEL
        elif action2 == 3:  # right
            self.target_linear_x2 = 0.0
            self.target_linear_y2 = -MAX_LIN_VEL
        else:  # stop
            self.target_linear_x2 = 0.0
            self.target_linear_y2 = 0.0

        # Create and publish twist messages
        twist1 = Twist()
        twist2 = Twist()
        
        # Robot1 movement
        twist1.linear.x = self.target_linear_x1
        twist1.linear.y = self.target_linear_y1
        twist1.linear.z = 0.0
        twist1.angular.x = 0.0
        twist1.angular.y = 0.0
        twist1.angular.z = 0.0
        
        # Robot2 movement
        twist2.linear.x = self.target_linear_x2
        twist2.linear.y = self.target_linear_y2
        twist2.linear.z = 0.0
        twist2.angular.x = 0.0
        twist2.angular.y = 0.0
        twist2.angular.z = 0.0

        # Publish commands
        try:
            self.publisher_robot1.publish(twist1)
            self.publisher_robot2.publish(twist2)
        except Exception as e:
            self.get_logger().error(f'Error publishing velocities: {str(e)}')
            return None, None, 0, 0, True

        # Store current velocities for next iteration
        self.control_linear_x1 = self.target_linear_x1
        self.control_linear_y1 = self.target_linear_y1
        self.control_linear_x2 = self.target_linear_x2
        self.control_linear_y2 = self.target_linear_y2

        # Small delay for physics update
        time.sleep(0.02)
        
        # Calculate rewards and check termination
        reward1, reward2 = self.calculate_rewards()
        
        # Get new states
        state1, state2 = self.get_states()
        
        return state1, state2, reward1, reward2, self.done

    def get_states(self):
        # State includes positions and velocities
        state1 = np.array([
            self.robot1_pos[0], self.robot1_pos[1],
            self.ball_pos[0], self.ball_pos[1],
            self.robot2_pos[0], self.robot2_pos[1],
            self.control_linear_x1,
            self.control_linear_y1
        ])
        
        state2 = np.array([
            self.robot2_pos[0], self.robot2_pos[1],
            self.ball_pos[0], self.ball_pos[1],
            self.robot1_pos[0], self.robot1_pos[1],
            self.control_linear_x2,
            self.control_linear_y2
        ])
        
        return state1, state2

def main(args=None):
    print("============================================================================================")
    print("Starting Robot Arena Training with PPO...")
    print("============================================================================================")
    
    rclpy.init(args=args)
    
    try:
        # Initialize environment
        env = GazeboEnv('ppo_training_node')
        
        # Create executor for the environment node
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(env)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Training parameters
        state_dim = 8
        action_dim = 5  # forward, backward, left, right, stop
        max_episodes = 5000
        max_timesteps = 1000
        update_timestep = 2000
        
        # Initialize PPO agents
        ppo_agent1 = PPO(state_dim, action_dim)
        ppo_agent2 = PPO(state_dim, action_dim)
        
        # Logging setup
        log_dir = "PPO_logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        log_file = os.path.join(log_dir, f'training_log_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv')
        with open(log_file, 'w') as f:
            f.write('Episode,Step,Reward1,Reward2,BallHits,Robot1Lives,Robot2Lives\n')
        
        # Training loop
        timestep = 0
        for episode in range(max_episodes):
            state1, state2 = env.reset()
            episode_reward1 = 0
            episode_reward2 = 0
            
            for t in range(max_timesteps):
                timestep += 1
                
                # Select actions
                action1 = ppo_agent1.select_action(state1)
                action2 = ppo_agent2.select_action(state2)
                
                # Execute actions
                next_state1, next_state2, reward1, reward2, done = env.step(action1, action2)
                
                # Store transitions
                ppo_agent1.memory.rewards.append(reward1)
                ppo_agent1.memory.is_terminals.append(done)
                
                ppo_agent2.memory.rewards.append(reward2)
                ppo_agent2.memory.is_terminals.append(done)
                
                episode_reward1 += reward1
                episode_reward2 += reward2
                
                # Update if its time
                if timestep % update_timestep == 0:
                    ppo_agent1.update()
                    ppo_agent2.update()
                
                state1 = next_state1
                state2 = next_state2
                
                if done:
                    break
            
            # Logging
            with open(log_file, 'a') as f:
                f.write(f'{episode},{t},{episode_reward1:.3f},{episode_reward2:.3f},' 
                       f'{env.ball_hit_count},{env.robot1_life},{env.robot2_life}\n')
            
            # Print episode summary
            if episode % 10 == 0:
                print(f"Episode: {episode}, Steps: {t}, "
                      f"Robot1 Reward: {episode_reward1:.3f}, Robot2 Reward: {episode_reward2:.3f}, "
                      f"Ball Hits: {env.ball_hit_count}, Lives: {env.robot1_life}/{env.robot2_life}")
            
            # Save models periodically
            if episode % 100 == 0:
                save_dir = "PPO_models"
                if not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                ppo_agent1.save(os.path.join(save_dir, f'ppo_agent1_episode_{episode}.pth'))
                ppo_agent2.save(os.path.join(save_dir, f'ppo_agent2_episode_{episode}.pth'))
    
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    except Exception as e:
        print(f"Error occurred: {str(e)}")
    finally:
        # Clean shutdown
        if 'env' in locals():
            env.destroy_node()
        if 'executor' in locals():
            executor.shutdown()
        rclpy.shutdown()
        if 'executor_thread' in locals():
            executor_thread.join()

if __name__ == '__main__':
    main()