import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Parâmetros para configuração de pinos
        self.declare_parameter('pwm_pin', 12)  # Pino PWM padrão
        self.declare_parameter('in1_pin', 5)   # Pino IN1
        self.declare_parameter('in2_pin', 6)   # Pino IN2
        self.declare_parameter('vel_topic', 'wheel_vel')  # Tópico de velocidade

        # Obtenção dos parâmetros
        self.pwm_pin = self.get_parameter('pwm_pin').get_parameter_value().integer_value
        self.in1_pin = self.get_parameter('in1_pin').get_parameter_value().integer_value
        self.in2_pin = self.get_parameter('in2_pin').get_parameter_value().integer_value
        vel_topic = self.get_parameter('vel_topic').get_parameter_value().string_value

        # Inicializar pigpio e configurar os pinos
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Falha ao conectar ao daemon pigpiod")
        
        # Configuração dos pinos de direção e PWM
        self.pi.set_mode(self.pwm_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.in1_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.in2_pin, pigpio.OUTPUT)

        # Assina o tópico de velocidade
        self.subscription = self.create_subscription(
            Twist,
            vel_topic,
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg):
        # Lê a velocidade linear do comando
        velocidade = msg.linear.x

        # Define a direção e ajusta o duty cycle do PWM
        if velocidade >= 0:
            # Movimento para frente
            self.pi.write(self.in1_pin, 1)
            self.pi.write(self.in2_pin, 0)
        else:
            # Movimento para trás
            self.pi.write(self.in1_pin, 0)
            self.pi.write(self.in2_pin, 1)
            velocidade = -velocidade  # Converte para positivo para o duty cycle

        # Converte a velocidade para duty cycle PWM e aplica
        duty_cycle = self.velocidade_para_pwm(velocidade)
        self.pi.set_PWM_dutycycle(self.pwm_pin, duty_cycle)
        self.get_logger().info(f'PWM no pino {self.pwm_pin}: {duty_cycle}, direção: {"frente" if msg.linear.x >= 0 else "ré"}')

    def velocidade_para_pwm(self, velocidade):
        # Converte a velocidade desejada em um valor de duty cycle (0-255)
        pwm_max = 255  # PWM máximo em duty cycle
        velocidade_max = 1.0  # Velocidade máxima desejada (ajuste conforme necessário)
        
        # Limita a velocidade ao máximo e converte para duty cycle
        duty_cycle = min(max(int((velocidade / velocidade_max) * pwm_max), 0), pwm_max)
        return duty_cycle

    def destroy_node(self):
        # Para o motor e limpa as configurações do pino
        self.pi.set_PWM_dutycycle(self.pwm_pin, 0)
        self.pi.write(self.in1_pin, 0)
        self.pi.write(self.in2_pin, 0)
        self.pi.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
