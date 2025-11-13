import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Float32MultiArray, MultiArrayDimension
import pygame
import math
import numpy as np
from typing import Tuple, List
from dataclasses import dataclass
import json # Import the json library to read the config file
import os   # To build a path to the config file

# Adding soccer ball constants
BALL_DIAMETER = 22  # cm
BALL_MASS = 0.4  # kg
GRAVITY = 980  # cm/s²
AIR_RESISTANCE = 0.1
GROUND_DAMPING = 0.7
CATCH_RADIUS = 30  # cm from center of robot
POSSESSION_COOLDOWN = 0.05  # seconds after kick before allowing new possession
MAX_KICK_SPEED = 800  # cm/s
MIN_KICK_ANGLE = 0  # degrees from horizontal
MAX_KICK_ANGLE = 45  # degrees from horizontal

# Constants for the simulation
WHEEL_DIAMETER = 10  # cm
BOT_SIZE = 50  # cm
MIN_SEPARATION_DISTANCE = BOT_SIZE + 20 # Minimum distance between bot centers
TICKS_PER_REVOLUTION = 1750
BOT_MASS = 20  # kg
WHEEL_DISTANCE = BOT_SIZE / math.sqrt(2)  # Distance from center to wheel
MOMENT_OF_INERTIA = BOT_MASS * ((BOT_SIZE/100) ** 2) / 6  # Approximating as solid square
FPS = 40
MAX_MOTOR_FORCE = 20  # Newtons
KP = 0.1
KI = 0
KD = 0.0001
MAX_TICKS_PER_SECOND = 30000
TICKS_PER_METER = (100 * TICKS_PER_REVOLUTION) / (math.pi*WHEEL_DIAMETER)

@dataclass
class BallState:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    possessed_by: int = -1
    possession_cooldown: float = 0

class SoccerBall:
    def __init__(self, x: float, y: float):
        self.state = BallState(x=x, y=y, z=0, vx=0, vy=0, vz=0)

    def update(self, dt: float, robots: List['OmniwheelRobot'], field_width: float, field_height: float) -> None:
        if self.state.possession_cooldown > 0:
            self.state.possession_cooldown -= dt

        if self.state.possessed_by >= 0:
            # Update position with robot
            robot_index = self.state.possessed_by
            if robot_index < len(robots):
                robot = robots[robot_index]
                self.state.x = robot.x + CATCH_RADIUS * math.cos(robot.theta)
                self.state.y = robot.y + CATCH_RADIUS * math.sin(robot.theta)
                self.state.z = 0
                self.state.vx = robot.vx
                self.state.vy = robot.vy
                self.state.vz = 0
            else:
                self.state.possessed_by = -1
            return

        # Apply physics when not possessed
        self.state.vx *= (1 - AIR_RESISTANCE * dt)
        self.state.vy *= (1 - AIR_RESISTANCE * dt)
        self.state.vz -= GRAVITY * dt
        self.state.vz *= (1 - AIR_RESISTANCE * dt)

        # Update positions
        self.state.x += self.state.vx * dt
        self.state.y += self.state.vy * dt
        self.state.z += self.state.vz * dt

        # Ground collision
        if self.state.z < 0:
            self.state.z = 0
            self.state.vz = -self.state.vz * GROUND_DAMPING

        if self.state.z<0.1:
            self.state.vx *= (1-(1-GROUND_DAMPING) * dt)
            self.state.vy *= (1-(1-GROUND_DAMPING) * dt)

        # Check for robot possession
        if self.state.z <= BALL_DIAMETER/2 + 5:
            for i, robot in enumerate(robots):
                dx = self.state.x - robot.x
                dy = self.state.y - robot.y
                dist = math.sqrt(dx*dx + dy*dy)
                relative_angle = math.atan2(dy, dx) - robot.theta

                if dist < CATCH_RADIUS and self.state.possession_cooldown <= 0 and abs(relative_angle) < math.pi/3:
                    self.state.possessed_by = i
                    break
                elif dist < BOT_SIZE/2:
                    bounce_angle = math.atan2(dy, dx)
                    speed = math.sqrt(self.state.vx**2 + self.state.vy**2)
                    self.state.vx = speed * math.cos(bounce_angle) * GROUND_DAMPING
                    self.state.vy = speed * math.sin(bounce_angle) * GROUND_DAMPING

        # Field boundaries
        if self.state.x < 0:
            self.state.x = 0
            self.state.vx = -self.state.vx * GROUND_DAMPING
        elif self.state.x > field_width:
            self.state.x = field_width
            self.state.vx = -self.state.vx * GROUND_DAMPING

        if self.state.y < 0:
            self.state.y = 0
            self.state.vy = -self.state.vy * GROUND_DAMPING
        elif self.state.y > field_height:
            self.state.y = field_height
            self.state.vy = -self.state.vy * GROUND_DAMPING

    def kick(self, speed: float, vertical_angle: float, robots: List['OmniwheelRobot']) -> None:
        if self.state.possessed_by < 0:
            return

        robot = robots[self.state.possessed_by]
        robot_angle = robot.theta
        vertical_rad = math.radians(np.clip(vertical_angle, MIN_KICK_ANGLE, MAX_KICK_ANGLE))
        speed = min(speed, MAX_KICK_SPEED)
        horizontal_speed = speed * math.cos(vertical_rad)

        self.state.vx = horizontal_speed * math.cos(robot_angle)
        self.state.vy = horizontal_speed * math.sin(robot_angle)
        self.state.vz = speed * math.sin(vertical_rad)

        self.state.possessed_by = -1
        self.state.possession_cooldown = POSSESSION_COOLDOWN

    # FIX: The draw method now accepts offsets to correctly position the ball inside the centered field.
    def draw(self, screen: pygame.Surface, scale: float, offset_x: int, offset_y: int) -> None:
        # FIX: Apply the calculated offsets to the ball's screen coordinates.
        ball_x = int(self.state.x * scale) + offset_x
        ball_y = int(self.state.y * scale) + offset_y
        ball_radius = int((BALL_DIAMETER/2) * scale)

        # Draw actual ball with slight offset based on height
        shadow_x = ball_x
        shadow_y = int(ball_y - self.state.z * scale) # Z offset is relative to the already corrected y
        pygame.draw.circle(screen, (105, 105, 105), (shadow_x, shadow_y), ball_radius)

        pygame.draw.circle(screen, (255, 255, 0), (ball_x, ball_y), ball_radius)

class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, target: float, current: float, dt: float) -> float:
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class OmniwheelRobot:
    def __init__(self, team: str, robot_number: int, x: float, y: float, theta: float = 0, color: tuple = (255, 0, 0)):
        self.team = team
        self.robot_number = robot_number
        self.x = x
        self.y = y
        self.theta = theta
        self.color = color
        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.target_vx = 0
        self.target_vy = 0
        self.target_omega = 0
        self.ax = 0
        self.ay = 0
        self.alpha = 0
        self.desired_ticks = [0, 0, 0, 0]
        self.achieved_ticks = [0, 0, 0, 0]
        self.current_wheel_velocities = [0, 0, 0, 0]
        self.pid_controllers = [PIDController(KP, KI, KD) for _ in range(4)]
        self.wheel_angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        self.motor_forces = [0, 0, 0, 0]
        self.target_velocities = [0, 0, 0, 0]

    def set_velocity(self, vx: float, vy: float, omega: float):
        root2 = math.sqrt(2)
        wheel_velocities = [(vx + vy)/root2 + omega, (-vx + vy)/root2 + omega, (-vx - vy)/root2 + omega, (vx - vy)/root2 + omega]        #calcuate wheel velocities in ticks per second
        self.add_target_ticks(wheel_velocities)

    def add_target_ticks(self, ticks_per_second: List[float]):
        self.target_velocities = ticks_per_second
        for i in range(4):
            self.desired_ticks[i] += ticks_per_second[i] / FPS

    def update(self, dt: float):
        self.motor_forces = []
        for i in range(4):
            pid_output = self.pid_controllers[i].compute(self.target_velocities[i], self.current_wheel_velocities[i], dt)
            # print(f"current wheel velocity {i}: {self.current_wheel_velocities[i]:.2f}, target: {self.target_velocities[i]:.2f}, pid output: {pid_output:.2f}")
            force = np.clip(pid_output*MAX_MOTOR_FORCE/255, -MAX_MOTOR_FORCE, MAX_MOTOR_FORCE)
            self.motor_forces.append(force)

        fx = fy = torque = 0
        for i, force in enumerate(self.motor_forces):
            angle = self.wheel_angles[i]
            fx += force * math.cos(angle)
            fy += force * math.sin(angle)
            torque += force * WHEEL_DISTANCE/100

        self.ax = 100*fx / BOT_MASS
        self.ay = 100*fy / BOT_MASS
        self.alpha = torque / MOMENT_OF_INERTIA

        self.vx += self.ax * dt
        self.vy += self.ay * dt
        self.omega += self.alpha * dt

        world_vx = self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)
        world_vy = self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.theta += self.omega * dt

        wheel_radius = WHEEL_DIAMETER / 2
        for i in range(4):
            angle = self.wheel_angles[i]
            linear_velocity = (self.vx * math.cos(angle) + self.vy * math.sin(angle) + self.omega * WHEEL_DISTANCE)
            self.current_wheel_velocities[i] = (linear_velocity * TICKS_PER_REVOLUTION / (2 * math.pi * wheel_radius))
            self.achieved_ticks[i] += self.current_wheel_velocities[i] * dt

        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    # FIX: The draw method now accepts offsets to correctly position the robot inside the centered field.
    def draw(self, screen: pygame.Surface, scale: float, offset_x: int, offset_y: int):
        """Draw the robot on the screen"""
        # FIX: Apply the calculated offsets to the robot's screen coordinates.
        px = int(self.x * scale) + offset_x
        py = int(self.y * scale) + offset_y
        size = int(BOT_SIZE * scale)

        surface = pygame.Surface((size, size), pygame.SRCALPHA)
        pygame.draw.rect(surface, (*self.color, 128), (0, 0, size, size))
        pygame.draw.circle(surface, (255,255,255), (size, size/2), size/4, 0)

        font = pygame.font.Font(None, int(size/1.5))
        number_text = font.render(str(self.robot_number), True, (255, 255, 255))
        text_rect = number_text.get_rect(center=(size/2, size/2))
        surface.blit(number_text, text_rect)

        wheel_size = int(WHEEL_DIAMETER * scale)
        for i, angle in enumerate(self.wheel_angles):
            wheel_x = size/2 + WHEEL_DISTANCE*scale*math.cos(angle)
            wheel_y = size/2 - WHEEL_DISTANCE*scale*math.sin(angle)
            velocity_ratio = self.target_velocities[i] / MAX_TICKS_PER_SECOND
            if velocity_ratio > 0: color = (255, 0, 0)
            elif velocity_ratio < 0: color = (0, 0, 255)
            else: color = (128, 128, 128)
            pygame.draw.circle(surface, (0,0,0), (int(wheel_x), int(wheel_y)), wheel_size)

        rotated = pygame.transform.rotate(surface, -math.degrees(self.theta))
        screen.blit(rotated, (px - rotated.get_width()/2, py - rotated.get_height()/2))

class ROS2RobotSimulation(Node):
    def __init__(self):
        super().__init__('robot_simulation')

        num_our_team = 3
        num_opp_team = 3
        field_dims_m = [22, 14]
        
        self.obstacle_noise_base_meters = 0.01  # Default 1cm base noise
        self.obstacle_noise_per_meter = 0.03  # Default 3cm noise per meter

        config_path = 'src/baseStation_pkg/baseStation_pkg/config.json'
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
                num_our_team = int(config.get('num_our_team', 5))
                num_opp_team = int(config.get('num_opp_team', 5))
                field_dims_m = config.get('field_dimensions', [22, 14])
                
                # Load new distance-based noise parameters
                self.obstacle_noise_base_meters = float(config.get('camera_noise_base_meters', 0.00))
                self.obstacle_noise_per_meter = float(config.get('camera_noise_per_meter', 0.00))

                if not isinstance(field_dims_m, list) or len(field_dims_m) != 2:
                    self.get_logger().warn("'field_dimensions' is invalid. Using default [22, 14].")
                    field_dims_m = [22, 14]
            self.get_logger().info(f"Successfully loaded config from '{config_path}'")
        except FileNotFoundError:
            self.get_logger().warn(f"'{config_path}' not found. Using default team sizes (5v5) and field size (22x14m).")
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().error(f"Error parsing '{config_path}': {e}. Using defaults.")

        self.field_width = field_dims_m[0] * 100
        self.field_height = field_dims_m[1] * 100
        self.get_logger().info(f"Spawning {num_our_team} 'our' robots and {num_opp_team} 'opponent' robots.")
        self.get_logger().info(f"Field dimensions set to {self.field_width/100}m x {self.field_height/100}m.")
        
        # Add a log message to confirm noise settings
        if self.obstacle_noise_per_meter > 0 or self.obstacle_noise_base_meters > 0:
            self.get_logger().info(f"Obstacle noise enabled: {self.obstacle_noise_base_meters}m base + {self.obstacle_noise_per_meter} per meter.")
    
        pygame.init()

        # FIX: Dynamically resize the window to match the field's aspect ratio.
        # Set a fixed window width.
        fixed_width = 800
        
        # Handle the case of zero field width to prevent division errors.
        if self.field_width > 0:
            # Calculate the scale based on the fixed width.
            self.scale = fixed_width / self.field_width
            # Calculate the corresponding height to maintain the aspect ratio.
            dynamic_height = int(self.scale * self.field_height)
        else:
            # Fallback for invalid field dimensions
            self.scale = 1
            dynamic_height = 800

        self.width = fixed_width
        self.height = dynamic_height

        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("ROS2 Omniwheel Robot Simulation")
        self.clock = pygame.time.Clock()

        self.robots = []
        self.robot_namespaces = []
        self.cmd_vel_subs = []
        self.state_pubs = []
        self.odom_pubs = []
        self.obstacle_pubs = []

        for i in range(num_our_team):
            namespace = f'o{i+1}'
            self.robot_namespaces.append(namespace)
            x = self.field_width / 4
            y = self.field_height / (num_our_team + 1) * (i + 1) if num_our_team > 0 else self.field_height / 2
            robot = OmniwheelRobot(team='o', robot_number=(i + 1), x=x, y=y, color=(255, 0, 0))
            self.robots.append(robot)

        for i in range(num_opp_team):
            namespace = f'b{i+1}'
            self.robot_namespaces.append(namespace)
            x = self.field_width * 3 / 4
            y = self.field_height / (num_opp_team + 1) * (i + 1) if num_opp_team > 0 else self.field_height / 2
            robot = OmniwheelRobot(team='b', robot_number=(i + 1), x=x, y=y, color=(0, 0, 255))
            self.robots.append(robot)

        for i, robot in enumerate(self.robots):
            namespace = self.robot_namespaces[i]
            self.cmd_vel_subs.append(self.create_subscription(Twist, f'{namespace}/cmd_vel', lambda msg, idx=i: self.cmd_vel_callback(msg, idx), 10))
            self.odom_pubs.append(self.create_publisher(Odometry, f'{namespace}_odom', 10))
            self.state_pubs.append(self.create_publisher(Float32MultiArray, f'{namespace}_data', 10))
            self.obstacle_pubs.append(self.create_publisher(Float32MultiArray, f'{namespace}_obstacles', 10))

        self.ball = SoccerBall(self.field_width/2, self.field_height/2)
        self.ball_state_pub = self.create_publisher(Float32MultiArray, 'ball_data', 10)
        self.command_sub = self.create_subscription(String, 'simulation/command', self.command_callback, 10)
        self.status_pub = self.create_publisher(String, 'simulation/status', 10)
        self.create_timer(1.0/FPS, self.update_and_render)
    
    def handle_robot_collisions(self):
        """
        Prevents robots from overlapping by correcting their position and velocity.
        """
        num_robots = len(self.robots)
        for i in range(num_robots):
            for j in range(i + 1, num_robots):
                robot_a = self.robots[i]
                robot_b = self.robots[j]

                dx = robot_b.x - robot_a.x
                dy = robot_b.y - robot_a.y
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < MIN_SEPARATION_DISTANCE:
                    # 1. Positional Correction
                    overlap = MIN_SEPARATION_DISTANCE - dist
                    nx = dx / dist if dist != 0 else 1
                    ny = dy / dist if dist != 0 else 0
                    
                    robot_a.x -= overlap / 2 * nx
                    robot_a.y -= overlap / 2 * ny
                    robot_b.x += overlap / 2 * nx
                    robot_b.y += overlap / 2 * ny

                    # 2. Velocity Correction
                    vel_a_n = robot_a.vx * nx + robot_a.vy * ny
                    vel_b_n = robot_b.vx * nx + robot_b.vy * ny

                    if vel_a_n > 0:
                        robot_a.vx -= vel_a_n * nx
                        robot_a.vy -= vel_a_n * ny

                    if vel_b_n < 0:
                        robot_b.vx -= vel_b_n * nx
                        robot_b.vy -= vel_b_n * ny
                        
    def command_callback(self, msg: String):
        try:
            cmd_parts = msg.data.split()
            if cmd_parts[0] == "KICK":
                if len(cmd_parts) == 3 and self.ball.state.possessed_by >= 0:
                    speed, angle = float(cmd_parts[1]), float(cmd_parts[2])
                    self.ball.kick(speed, angle, self.robots)
            elif cmd_parts[0] == "PLACE":
                if len(cmd_parts) == 4:
                    x, y, z = float(cmd_parts[1])*100, float(cmd_parts[2])*100, float(cmd_parts[3])*100
                    self.ball.state.x = self.field_width/2 + x
                    self.ball.state.y = self.field_height/2 - y
                    self.ball.state.z = z
                    self.ball.state.vx = self.ball.state.vy = self.ball.state.vz = 0
                    self.ball.state.possessed_by = -1
        except (IndexError, ValueError) as e:
            self.get_logger().warn(f'Invalid command format: {e}')

    def publish_ball_state(self):
        state_msg = Float32MultiArray()
        state_msg.layout.dim = [MultiArrayDimension(label="state", size=7, stride=7)]
        possession_id = -1.0
        if self.ball.state.possessed_by >= 0 and self.robots[self.ball.state.possessed_by].team == 'o':
            possession_id = float(self.robots[self.ball.state.possessed_by].robot_number - 1)
        try:
            state_data = [
                float((self.ball.state.x - self.field_width/2)/100),
                float((self.field_height/2 - self.ball.state.y)/100),
                float(self.ball.state.z/100),
                float(self.ball.state.vx/100),
                float(self.ball.state.vy/100),
                float(self.ball.state.vz/100),
                possession_id
            ]
            state_msg.data = [np.clip(val, -3.4e38, 3.4e38) for val in state_data]
        except (ValueError, TypeError) as e:
            self.get_logger().error(f'Error converting ball state data: {e}')
            state_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0]
        self.ball_state_pub.publish(state_msg)

    def publish_obstacles(self, robot_index: int):
        obstacles_msg = Float32MultiArray()
        obstacle_data = []
        
        # Get the robot that is "seeing" the obstacles
        sensing_robot = self.robots[robot_index]

        for i, robot in enumerate(self.robots): # 'robot' is the obstacle
            if i != robot_index:
                
                # Calculate distance from sensor to obstacle (in cm first)
                dx_cm = robot.x - sensing_robot.x
                dy_cm = robot.y - sensing_robot.y
                distance_cm = math.sqrt(dx_cm**2 + dy_cm**2)
                distance_m = distance_cm / 100.0
                
                # Calculate noise standard deviation (sigma) based on distance
                sigma = self.obstacle_noise_base_meters + (distance_m * self.obstacle_noise_per_meter)
                
                # Generate the noise
                noise_x = np.random.normal(0.0, sigma)
                noise_y = np.random.normal(0.0, sigma)

                # Get "true" position in meters, relative to center
                true_x_pos = (robot.x - self.field_width/2) / 100.0
                true_y_pos = (self.field_height/2 - robot.y) / 100.0

                # Apply noise
                noisy_x_pos = true_x_pos + noise_x
                noisy_y_pos = true_y_pos + noise_y

                obstacle_data.extend([noisy_x_pos, noisy_y_pos])
        
        msg_data = [float(len(obstacle_data) // 2)] + obstacle_data
        obstacles_msg.layout.dim = [MultiArrayDimension(label="obstacles", size=len(msg_data), stride=len(msg_data))]
        obstacles_msg.data = msg_data
        self.obstacle_pubs[robot_index].publish(obstacles_msg)

    def update_and_render(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                rclpy.shutdown()
                return

        dt = 1.0/FPS
        for i, robot in enumerate(self.robots):
            robot.set_velocity(robot.target_vx, robot.target_vy, robot.target_omega)
            robot.update(dt)

        # Handle collisions after all robots have moved for this frame
        self.handle_robot_collisions()

        # Now publish the corrected states
        for i, robot in enumerate(self.robots):
            self.publish_robot_state(robot, i)
            self.publish_obstacles(i)

        status_msg = String(data="Simulation running")
        self.status_pub.publish(status_msg)

        self.ball.update(1.0/FPS, self.robots, self.field_width, self.field_height)
        self.publish_ball_state()

        self.screen.fill((0, 100, 0)) # Green Field

        # Since the window is the field, the offsets will now be zero, but the logic remains robust.
        field_width_px = int(self.field_width * self.scale)
        field_height_px = int(self.field_height * self.scale)
        start_x = (self.width - field_width_px) // 2 # This will be 0
        start_y = (self.height - field_height_px) // 2 # This will be 0

        # Pass the offsets (which are now 0) to the draw functions.
        self.draw_field(start_x, start_y, field_width_px, field_height_px)
        for robot in self.robots:
            robot.draw(self.screen, self.scale, start_x, start_y)
        self.ball.draw(self.screen, self.scale, start_x, start_y)

        # Draw height bar (this is relative to the window, so no offset needed)
        bar_height, bar_width = 200, 20
        bar_x, bar_y = self.width - 40, self.height - 20 - bar_height
        max_height = 300
        pygame.draw.rect(self.screen, (64, 64, 64), (bar_x, bar_y, bar_width, bar_height))
        height_percentage = min(self.ball.state.z / max_height, 1.0)
        fill_height = int(bar_height * height_percentage)
        pygame.draw.rect(self.screen, (0, 255, 0), (bar_x, bar_y + bar_height - fill_height, bar_width, fill_height))
        font = pygame.font.Font(None, 24)
        height_text = font.render(f'Ball:{self.ball.state.z:.1f} cm', True, (255, 255, 255))
        self.screen.blit(height_text, (bar_x - 60, bar_y + bar_height - fill_height))

        pygame.display.flip()
        self.clock.tick(FPS)

    # FIX: This function now takes the pre-calculated pixel values for simplicity.
    def draw_field(self, start_x, start_y, field_width_px, field_height_px):
        """Draw the field with boundaries"""
        # FIX: Simplified back to only drawing the lines. The green background is handled by screen.fill().
        # Draw the white boundary lines.
        pygame.draw.rect(self.screen, (255, 255, 255), (start_x, start_y, field_width_px, field_height_px), 2)
        
        # Center line
        pygame.draw.line(self.screen, (255, 255, 255), (start_x + field_width_px//2, start_y), (start_x + field_width_px//2, start_y + field_height_px), 2)
        
        # Center circle
        pygame.draw.circle(self.screen, (255, 255, 255), (start_x + field_width_px//2, start_y + field_height_px//2), int(50 * self.scale), 2)



    def cmd_vel_callback(self, msg: Twist, robot_index: int):
        theta = self.robots[robot_index].theta
        global_vx = msg.linear.x * TICKS_PER_METER
        global_vy = -msg.linear.y * TICKS_PER_METER
        local_vx = global_vx * math.cos(theta) + global_vy * math.sin(theta)
        local_vy = -global_vx * math.sin(theta) + global_vy * math.cos(theta)
        omega = -msg.angular.z * WHEEL_DISTANCE * TICKS_PER_METER/100
        self.robots[robot_index].target_vx = local_vx
        self.robots[robot_index].target_vy = local_vy
        self.robots[robot_index].target_omega = omega

    def publish_robot_state(self, robot: OmniwheelRobot, robot_index: int):
        current_time = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = f"{self.robot_namespaces[robot_index]}_base_link"
        odom.header.stamp = current_time
        odom.pose.pose.position.x = robot.x / 100.0
        odom.pose.pose.position.y = robot.y / 100.0
        odom.pose.pose.orientation.z = math.sin(robot.theta/2)
        odom.pose.pose.orientation.w = math.cos(robot.theta/2)
        odom.twist.twist.linear.x = robot.vx / 100.0
        odom.twist.twist.linear.y = -robot.vy / 100.0
        odom.twist.twist.angular.z = -robot.omega
        self.odom_pubs[robot_index].publish(odom)

        state_msg = Float32MultiArray()
        state_msg.layout.dim = [MultiArrayDimension(label="state", size=7, stride=7)]
        state_msg.data = [
            (robot.x - self.field_width/2)/100,
            (self.field_height/2 - robot.y)/100,
            -robot.theta,
            -robot.achieved_ticks[0],
            -robot.achieved_ticks[3],
            -robot.achieved_ticks[1],
            -robot.achieved_ticks[2]
        ]
        self.state_pubs[robot_index].publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    sim_node = ROS2RobotSimulation()
    try:
        rclpy.spin(sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()