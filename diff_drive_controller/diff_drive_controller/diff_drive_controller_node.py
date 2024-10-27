import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf_transformations
from tf2_ros import TransformBroadcaster
import math
import time

class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller_node')

        # Parâmetros do robô
        self.declare_parameter('wheel_base', 0.3)  # Distância entre as rodas
        self.declare_parameter('wheel_radius', 0.068)  # Raio das rodas

        # Obtenção dos parâmetros
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Estado inicial da odometria
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Assinatura do tópico `cmd_vel` para controlar o movimento
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Publicador de odometria e transformador
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Assinatura do `joint_states` para obter dados de posição e velocidade de cada roda
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Variáveis para armazenar a velocidade de cada roda
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0

    def cmd_vel_callback(self, msg):
        # Recebe o comando de velocidade (cmd_vel) e atualiza as velocidades das rodas
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Converte cmd_vel em velocidades para cada roda
        self.left_wheel_vel = (linear_velocity - angular_velocity * self.wheel_base / 2) / self.wheel_radius
        self.right_wheel_vel = (linear_velocity + angular_velocity * self.wheel_base / 2) / self.wheel_radius

    def joint_state_callback(self, msg):
        # Atualiza a posição do robô com base nas informações do `joint_state`
        left_wheel_pos = msg.position[0]
        right_wheel_pos = msg.position[1]
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Converte para segundos
        self.last_time = current_time

        # Calcula a velocidade linear e angular do robô
        linear_velocity = (self.right_wheel_vel + self.left_wheel_vel) * self.wheel_radius / 2
        angular_velocity = (self.right_wheel_vel - self.left_wheel_vel) * self.wheel_radius / self.wheel_base

        # Atualiza a posição (x, y) e orientação (theta)
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Publica a odometria
        self.publish_odometry(linear_velocity, angular_velocity, current_time)

    def publish_odometry(self, linear_velocity, angular_velocity, current_time):
        # Criação da mensagem de odometria
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Posição
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # Velocidade linear e angular
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity

        # Publica a mensagem de odometria
        self.odom_pub.publish(odom)

        # Publica o transform `tf`
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = odom.pose.pose.orientation.z
        t.transform.rotation.w = odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
