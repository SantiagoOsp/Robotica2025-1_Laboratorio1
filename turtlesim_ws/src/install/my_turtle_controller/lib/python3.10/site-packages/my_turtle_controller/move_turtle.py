import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pinput import keyboard 

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Crear un "listener" del teclado
        listener = keyboard.Listener(
            on_press = self.boton_presionado, # Llamar la función boton_presionado cuando se presione el botón
            on_release = self.boton_liberado) # Llamar la función boton_liberado cuando se libere el botón
        # Inicializar el "listener"
        listener.start()

    def boton_presionado(self, key)
        if key == keyboard.Key.up
            self.mover_hacia_adelante()
    
    def boton_liberado(self, key)
        if key == keyboard.Key.up
            self.detener_movimiento()

    def detener_movimiento(self)
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Movimiento detenido")
    
    def mover_hacia_adelante(self)
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info("Moviendo hacia adelante con la flecha arriba")

    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()