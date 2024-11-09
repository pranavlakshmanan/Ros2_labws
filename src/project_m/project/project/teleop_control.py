import os
import select
import sys
import rclpy

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 0.5
MAX_ANG_VEL = 1.57

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBots!
---------------------------
Robot1 (WASDQE):             Robot2 (IJKLUO):
   q    w    e                  u    i    o
   a    s    d                  j    k    l
        x                            ,

w/x : increase/decrease linear velocity (Robot1)    i/, : increase/decrease linear velocity (Robot2)
a/d : increase/decrease linear velocity (Robot1)    j/l : increase/decrease linear velocity (Robot2)
q/e : increase/decrease angular velocity (Robot1)   u/o : increase/decrease angular velocity (Robot2)
s : force stop Robot1                              k : force stop Robot2
space key : force stop both robots
CTRL-C to quit
"""

e = """
Communications Failed
"""

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(target_linear_velocity1, target_angular_velocity1, 
               target_linear_velocity2, target_angular_velocity2):
    print('Robot1:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity1, target_angular_velocity1))
    print('Robot2:\tlinear velocity {0}\t angular velocity {1} '.format(
        target_linear_velocity2, target_angular_velocity2))

def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input
    return output

def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel
    return input_vel

def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)

def main():
    move_vector1 = 0
    move_vector2 = 0
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('dual_robot_teleop_key')
    pub1 = node.create_publisher(Twist, '/Robot1/cmd_vel', qos)
    pub2 = node.create_publisher(Twist, '/Robot2/cmd_vel', qos)

    status = 0
    target_linear_velocity1 = target_linear_velocity2 = 0.0
    target_angular_velocity1 = target_angular_velocity2 = 0.0
    control_linear_velocity1 = control_linear_velocity2 = 0.0
    control_angular_velocity1 = control_angular_velocity2 = 0.0

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            
            # Robot1 controls (WASDQE)
            if key == 'w':
                target_linear_velocity1 = check_linear_limit_velocity(target_linear_velocity1 + LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector1 = 0
            elif key == 'x':
                target_linear_velocity1 = check_linear_limit_velocity(target_linear_velocity1 - LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector1 = 0
            elif key == 'a':
                target_linear_velocity1 = check_linear_limit_velocity(target_linear_velocity1 + LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector1 = 1
            elif key == 'd':
                target_linear_velocity1 = check_linear_limit_velocity(target_linear_velocity1 - LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector1 = 1
            elif key == 'q':
                target_angular_velocity1 = check_angular_limit_velocity(target_angular_velocity1 + ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'e':
                target_angular_velocity1 = check_angular_limit_velocity(target_angular_velocity1 - ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == 's':
                target_linear_velocity1 = 0.0
                control_linear_velocity1 = 0.0
                target_angular_velocity1 = 0.0
                control_angular_velocity1 = 0.0
            
            # Robot2 controls (IJKLUO)
            elif key == 'i':
                target_linear_velocity2 = check_linear_limit_velocity(target_linear_velocity2 + LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector2 = 0
            elif key == ',':
                target_linear_velocity2 = check_linear_limit_velocity(target_linear_velocity2 - LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector2 = 0
            elif key == 'j':
                target_linear_velocity2 = check_linear_limit_velocity(target_linear_velocity2 + LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector2 = 1
            elif key == 'l':
                target_linear_velocity2 = check_linear_limit_velocity(target_linear_velocity2 - LIN_VEL_STEP_SIZE)
                status = status + 1
                move_vector2 = 1
            elif key == 'u':
                target_angular_velocity2 = check_angular_limit_velocity(target_angular_velocity2 + ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'o':
                target_angular_velocity2 = check_angular_limit_velocity(target_angular_velocity2 - ANG_VEL_STEP_SIZE)
                status = status + 1
            elif key == 'k':
                target_linear_velocity2 = 0.0
                control_linear_velocity2 = 0.0
                target_angular_velocity2 = 0.0
                control_angular_velocity2 = 0.0
            
            # Global controls
            elif key == ' ':
                # Stop both robots
                target_linear_velocity1 = target_linear_velocity2 = 0.0
                control_linear_velocity1 = control_linear_velocity2 = 0.0
                target_angular_velocity1 = target_angular_velocity2 = 0.0
                control_angular_velocity1 = control_angular_velocity2 = 0.0
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            # Update Robot1 velocities
            twist1 = Twist()
            control_linear_velocity1 = make_simple_profile(
                control_linear_velocity1,
                target_linear_velocity1,
                (LIN_VEL_STEP_SIZE / 2.0))

            if move_vector1 == 0:
                twist1.linear.x = control_linear_velocity1
                twist1.linear.y = 0.0
            elif move_vector1 == 1:
                twist1.linear.x = 0.0
                twist1.linear.y = control_linear_velocity1
            twist1.linear.z = 0.0

            control_angular_velocity1 = make_simple_profile(
                control_angular_velocity1,
                target_angular_velocity1,
                (ANG_VEL_STEP_SIZE / 2.0))
            
            twist1.angular.x = 0.0
            twist1.angular.y = 0.0
            twist1.angular.z = control_angular_velocity1

            # Update Robot2 velocities
            twist2 = Twist()
            control_linear_velocity2 = make_simple_profile(
                control_linear_velocity2,
                target_linear_velocity2,
                (LIN_VEL_STEP_SIZE / 2.0))

            if move_vector2 == 0:
                twist2.linear.x = control_linear_velocity2
                twist2.linear.y = 0.0
            elif move_vector2 == 1:
                twist2.linear.x = 0.0
                twist2.linear.y = control_linear_velocity2
            twist2.linear.z = 0.0

            control_angular_velocity2 = make_simple_profile(
                control_angular_velocity2,
                target_angular_velocity2,
                (ANG_VEL_STEP_SIZE / 2.0))
            
            twist2.angular.x = 0.0
            twist2.angular.y = 0.0
            twist2.angular.z = control_angular_velocity2

            # Publish velocities
            pub1.publish(twist1)
            pub2.publish(twist2)

            if status % 5 == 0:
                print_vels(control_linear_velocity1, control_angular_velocity1,
                          control_linear_velocity2, control_angular_velocity2)

    except Exception as e:
        print(e)

    finally:
        # Stop both robots
        twist1 = Twist()
        twist2 = Twist()
        pub1.publish(twist1)
        pub2.publish(twist2)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__ == '__main__':
    main()