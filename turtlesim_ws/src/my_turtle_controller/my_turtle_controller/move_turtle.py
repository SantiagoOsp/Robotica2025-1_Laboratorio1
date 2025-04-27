import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from pynput import keyboard # Agregar la libreria de teclado
import time # Agregar libreria de tiempo
from turtlesim.srv import TeleportAbsolute # Agregar servicio de movimiento absoluto
from turtlesim.msg import Pose # Agregar el mensaje de la pose

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        # Ser publicador de comandos de velocidad
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Guardar la última pose conocida
        self.pose = Pose()
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.actualizar_pose,
            10
        )

        # Crear un "listener" del teclado
        listener = keyboard.Listener(
            on_press = self.boton_presionado, # Llamar la función boton_presionado cuando se presione el botón
            on_release = self.boton_liberado) # Llamar la función boton_liberado cuando se libere el botón
        # Inicializar el "listener"
        listener.start()

    def boton_presionado(self, key):
        # print(f"Tecla presionada: {key}")
        try:
            match key:
                case keyboard.Key.up:
                    self.mover_hacia_adelante(1, 1.0)
                    self.get_logger().info("\u2191") # Imprime un '↑'
                case keyboard.Key.down:
                    self.mover_hacia_atras(1, 1.0)
                    self.get_logger().info("\u2193") # Imprime un '↓'
                case keyboard.Key.left:
                    self.rotar_antihorario90(1)
                    self.get_logger().info("\u2190") # Imprime un '←'
                case keyboard.Key.right:
                    self.rotar_horario90(1)
                    self.get_logger().info("\u2192") # Imprime un '→'                    
                case _:
                    try:
                        self.dibujar(key.char.lower())
                    except AttributeError:
                            pass
        except AttributeError:
            pass
    
    def boton_liberado(self, key):
        return

    def actualizar_pose(self, msg):
        self.pose = msg

    def orientar(self, myTheta):
        client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

        request = TeleportAbsolute.Request()
        request.x = self.pose.x
        request.y = self.pose.y
        request.theta = myTheta

        future = client.call_async(request)
        future.add_done_callback(self.orientacion_callback)
        time.sleep(1)

    def rotar_horario90(self, n):
        for i in range(0, n):
            client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

            request = TeleportAbsolute.Request()
            request.x = self.pose.x
            request.y = self.pose.y
            request.theta = self.pose.theta - 3.1415926535897932384626433 / 2

            future = client.call_async(request)
            future.add_done_callback(self.orientacion_callback)
        time.sleep(1)

    def rotar_antihorario90(self, n):
        for i in range(0, n):
            client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

            request = TeleportAbsolute.Request()
            request.x = self.pose.x
            request.y = self.pose.y
            request.theta = self.pose.theta + 3.1415926535897932384626433 / 2

            future = client.call_async(request)
            future.add_done_callback(self.orientacion_callback)
        time.sleep(1)

    def orientacion_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            pass

    def detener_movimiento(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
    
    def mover_hacia_adelante(self, n, vel):
        for i in range(0, n):
            msg = Twist()
            msg.linear.x = vel
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(1)
            self.detener_movimiento()

    def mover_hacia_atras(self, n, vel):
        for i in range(0, n):
            msg = Twist()
            msg.linear.x = -vel
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            time.sleep(1)
            self.detener_movimiento()
    
    def curva_derecha(self, n, velx):
        for i in range(0, n):
            msg = Twist()
            msg.linear.x = velx
            msg.angular.z = -3.1415926535897932384626433 / 2
            self.publisher_.publish(msg)
            time.sleep(1)
            self.detener_movimiento()

    def curva_izquierda(self, n, velx):
        for i in range(0, n):
            msg = Twist()
            msg.linear.x = velx
            msg.angular.z = 3.1415926535897932384626433 / 2
            self.publisher_.publish(msg)
            time.sleep(1)
            self.detener_movimiento()

    def dibujar(self, letra):
        msg1 = Twist() # Crear mensaje
        msg1.angular.z = 0.0
        self.orientar(0.0) # Orientar para dibujar

        match letra:
            case 'j':
                self.get_logger().info("Dibujando letra J")
                # --
                self.mover_hacia_adelante(2, 1.0)
                self.mover_hacia_atras(1, 1.0)
                # |
                self.rotar_horario90(1)
                self.mover_hacia_adelante(2, 1.0)
                # ( )
                self.curva_derecha(2, 1.0)
            
            case 'd':
                self.get_logger().info("Dibujando letra D")
                # -
                self.mover_hacia_adelante(1, 1.0)
                # )
                self.curva_derecha(1, 1.0)
                self.orientar(-3.1415926535897932384626433 / 2)
                # |
                self.mover_hacia_adelante(2, 1.0)
                # )
                self.curva_derecha(1, 1.0)
                self.orientar(3.1415926535897932384626433)
                # -
                self.mover_hacia_adelante(1, 1.0)
                # |
                self.orientar(3.1415926535897932384626433 / 2)
                self.mover_hacia_adelante(1, 3.3)

            case 't':
                self.get_logger().info("Dibujando letra T")
                # --
                self.mover_hacia_adelante(2, 1.0)
                self.mover_hacia_atras(1, 1.0)
                # |
                self.rotar_horario90(1)
                self.mover_hacia_adelante(2, 1.0)
                
            case 'c':
                self.get_logger().info("Dibujando letra C")
                # (
                self.orientar(3.1415926535897932384626433)
                self.curva_izquierda(1, 2.0)
                # |
                self.orientar(-3.1415926535897932384626433 / 2)
                self.mover_hacia_adelante(1, 0.5)
                # (
                self.curva_izquierda(1, 2.0)
            case 's':
                self.get_logger().info("Dibujando letra S")
                self.orientar(3.1415926535897932384626433)
                self.curva_izquierda(2, 1.2)
                self.curva_derecha(2, 1.2)

                pass
            case 'o':
                self.get_logger().info("Dibujando letra O")
                self.curva_izquierda(4, 2.0)
            case _:
                self.get_logger().info("Letra no reconocida para dibujo")



def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()