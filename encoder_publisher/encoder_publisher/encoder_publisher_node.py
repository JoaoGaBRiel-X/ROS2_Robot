import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import pigpio
import time
import math
import collections

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')

        # Parâmetros de configuração dos encoders e rodas
        self.declare_parameter('left_encoder_pin', 17)
        self.declare_parameter('right_encoder_pin', 27)
        self.declare_parameter('pulses_per_revolution', 20)
        self.declare_parameter('wheel_radius', 0.068)
        
        # Obter valores dos parâmetros
        self.left_encoder_pin = self.get_parameter('left_encoder_pin').get_parameter_value().integer_value
        self.right_encoder_pin = self.get_parameter('right_encoder_pin').get_parameter_value().integer_value
        self.pulses_per_revolution = self.get_parameter('pulses_per_revolution').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value

        # Configuração de variáveis para contagem de pulsos e cálculos
        self.left_ticks = 0
        self.right_ticks = 0
        self.left_position = 0.0  # Posição acumulada da roda esquerda
        self.right_position = 0.0  # Posição acumulada da roda direita
        self.last_time = time.time()
        self.distance_per_pulse = (2 * math.pi * self.wheel_radius) / self.pulses_per_revolution

        # Inicializar pigpio e configurar os pinos dos encoders
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Erro ao conectar ao daemon pigpiod")
        
        # Configuração dos pinos dos encoders como entrada com pull-up
        self.pi.set_mode(self.left_encoder_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.left_encoder_pin, pigpio.PUD_UP)
        self.pi.callback(self.left_encoder_pin, pigpio.RISING_EDGE, self.left_encoder_callback)

        self.pi.set_mode(self.right_encoder_pin, pigpio.INPUT)
        self.pi.set_pull_up_down(self.right_encoder_pin, pigpio.PUD_UP)
        self.pi.callback(self.right_encoder_pin, pigpio.RISING_EDGE, self.right_encoder_callback)

        # Variáveis para armazenar a direção com base nos valores de velocidade de cada roda
        self.left_direction = 1
        self.right_direction = 1

        # Assinar tópicos de velocidade de cada roda (usando Twist)
        self.create_subscription(Twist, 'wheel_vel_left', self.left_wheel_velocity_callback, 10)
        self.create_subscription(Twist, 'wheel_vel_right', self.right_wheel_velocity_callback, 10)

        # Fila para média móvel de velocidade com janela de 8 leituras
        self.left_velocity_window = collections.deque(maxlen=8)
        self.right_velocity_window = collections.deque(maxlen=8)

        # Publicador de estado das juntas
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_encoder_data)

    def left_wheel_velocity_callback(self, msg):
        # Define a direção com base no valor de linear.x da roda esquerda
        self.left_direction = 1 if msg.linear.x >= 0 else -1

    def right_wheel_velocity_callback(self, msg):
        # Define a direção com base no valor de linear.x da roda direita
        self.right_direction = 1 if msg.linear.x >= 0 else -1

    def left_encoder_callback(self, gpio, level, tick):
        # Incrementa os pulsos do encoder
        self.left_ticks += 1

    def right_encoder_callback(self, gpio, level, tick):
        # Incrementa os pulsos do encoder
        self.right_ticks += 1

    def publish_encoder_data(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Cálculo da velocidade instantânea em metros por segundo
        left_velocity = (self.left_ticks * self.distance_per_pulse) / dt if dt > 0 else 0.0
        right_velocity = (self.right_ticks * self.distance_per_pulse) / dt if dt > 0 else 0.0

        # Adiciona a velocidade atual à janela de média móvel
        self.left_velocity_window.append(left_velocity)
        self.right_velocity_window.append(right_velocity)

        # Calcula a média móvel para suavizar as velocidades
        left_velocity_avg = sum(self.left_velocity_window) / len(self.left_velocity_window)
        right_velocity_avg = sum(self.right_velocity_window) / len(self.right_velocity_window)

        # Atualiza a posição acumulada com base na direção
        self.left_position += self.left_direction * self.left_ticks * self.distance_per_pulse
        self.right_position += self.right_direction * self.right_ticks * self.distance_per_pulse

        # Preencher a mensagem JointState
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [self.left_position, self.right_position]
        msg.velocity = [left_velocity_avg * self.left_direction, right_velocity_avg * self.right_direction]

        # Publica a mensagem
        self.publisher.publish(msg)
        
        # Reinicia as contagens de pulsos para a próxima leitura
        self.left_ticks = 0
        self.right_ticks = 0

    def destroy_node(self):
        # Para o uso do pino e encerra o pigpio
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
