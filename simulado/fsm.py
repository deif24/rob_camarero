#!/usr/bin/env python3
import rospy
import smach_ros
from smach import State, StateMachine
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Tópics relacionados con interfaz
TOPIC_SELECCION_MESAS = "/seleccion_mesas"
TOPIC_BOTON_PEDIDOS = "/boton_pedidos"

# Topics de navegación
TOPIC_NAVEGACION_START = "/navegacion_start"
TOPIC_NAVEGACION_END = "/navegacion_end"

# Topics de visual
TOPIC_VISUAL_START = "/manos_start"
TOPIC_VISUAL_END = "/manos_end"

# Topics de pedidos
TOPIC_PEDIDOS_START = "/pedidos_start"
TOPIC_PEDIDOS_END = "/pedidos_end"

# Variables globales
mesa_seleccionada = 0
estado_navegacion = 0

class ReposoState(State):
    def __init__(self):
        State.__init__(self, outcomes=['task'])
        self.pubVisual = rospy.Publisher(TOPIC_VISUAL_START, Int32, queue_size=5)  
        self.pubCmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.subStart = None
        self.subVisual = None
        self.subOdom = None
        self.task = False
        self.visual_finished = False

        self.twist_msg = Twist()
        self.twist_msg.angular.z = 1.0
        self.orientation_z = None

    def execute(self, userdata):
        rospy.loginfo("Estado: ReposoState")
        self.task = False
        self.visual_finished = False
        self.twist_msg.angular.z = 1.0

        # Suscripción a topics
        self.subStart = rospy.Subscriber(TOPIC_SELECCION_MESAS, Int32, self.seleccion_mesas_callback)
        self.subVisual = rospy.Subscriber(TOPIC_VISUAL_END, Int32, self.visual_callback)
        self.subOdom = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Iniciamos el nodo de deteccion de manos
        rospy.sleep(1)
        self.pubVisual.publish(1)
        
        rate = rospy.Rate(10)

        # Mientras no pase nada el robot gira buscando manos levantadas o esperando a que se pulse el botón de mesa
        while not (self.task or self.visual_finished):
            self.pubCmdVel.publish(self.twist_msg)

        # Detener el movimiento
        self.twist_msg.angular.z = 0.0
        self.pubCmdVel.publish(self.twist_msg)

        # Desuscribirse después de terminar
        if self.subStart:
            self.subStart.unregister()
        if self.subVisual:
            self.subVisual.unregister()
        if self.subOdom:
            self.subOdom.unregister()

        return 'task'

    # Callback que selecciona mesa en funcion del boton pulsado en la interfaz
    def seleccion_mesas_callback(self, msg):
        global mesa_seleccionada
        if msg.data > 0:  # Validar que mesa seleccionada sea válida
            mesa_seleccionada = msg.data
            rospy.loginfo("Mesa seleccionada: {}".format(mesa_seleccionada))
            self.task = True

    # Callback que selecciona mesa en funcion de la orientacion del robot al detectar una mano levantada
    def visual_callback(self, msg):
        global mesa_seleccionada
        if msg.data:
            if self.orientation_z >= 1.57 and self.orientation_z < 3.15:
                mesa_seleccionada = 1
            elif self.orientation_z >= -3.15 and self.orientation_z < -1.57:
                mesa_seleccionada = 2
            elif self.orientation_z >= 0 and self.orientation_z < 1.57:
                mesa_seleccionada = 3
            elif self.orientation_z >= -1.57 and self.orientation_z < 0:
                mesa_seleccionada = 4

            rospy.loginfo("Mano levantada detectada en la mesa {}".format(mesa_seleccionada))
            self.visual_finished = True

    # Callback para obtener la orientacion del robot en radianes
    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        quaternion = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        # Convertir de cuaternios a ángulos de Euler (roll, pitch, yaw)
        euler_angles = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler_angles
        self.orientation_z = yaw # En radianes

# Estado para navegar a las diferentes posiciones del "restaurante"
class NavigationState(State):
    def __init__(self):
        State.__init__(self, outcomes=['to_voice','to_reposo'])
        self.pubNavStart = rospy.Publisher(TOPIC_NAVEGACION_START, Int32, queue_size=5)
        self.subNavEnd = None
        self.nav_finished = False

    def execute(self, userdata):
        global mesa_seleccionada, estado_navegacion
        rospy.loginfo("Estado: NavigationState")
        self.nav_finished = False

        # Suscripción a topics
        self.subNavEnd = rospy.Subscriber(TOPIC_NAVEGACION_END, Int32, self.nav_end_callback)

        # Navegación condicional
        # Si tiene que ir de un lugar hacia una de las mesas
        if estado_navegacion == 0 or estado_navegacion == 2:
            rospy.loginfo("Navegación a mesa")
            self.pubNavStart.publish(mesa_seleccionada)
            rate = rospy.Rate(10)

        # Si tiene que ir a la cocina
        elif estado_navegacion == 1:
            rospy.loginfo("Navegacion a cocina")
            self.pubNavStart.publish(10)
            rate = rospy.Rate(10)

        # Si tiene que ir al reposo
        elif estado_navegacion == 3:
            rospy.loginfo("Navegación a reposo")
            self.pubNavStart.publish(0)
            rate = rospy.Rate(10)

        while not self.nav_finished:
            rate.sleep()

        # Desuscribirse después de terminar
        if self.subNavEnd:
            self.subNavEnd.unregister()

        # Se cambia a un estado u otro en función de en qué punto del programa estemos
        if estado_navegacion == 3:
            estado_navegacion = 0 
            return 'to_reposo'
        else:
            estado_navegacion += 1
            return 'to_voice'

    def nav_end_callback(self, msg):
        if msg.data == 1:  # Navegación completada
            self.nav_finished = True
            rospy.loginfo("Navegación terminada")


# Estado encargado de registrar los pedidos y confirmar por voz otras condiciones
class VoiceState(State):
    def __init__(self):
        State.__init__(self, outcomes=['voice_finished'])
        self.pubVoiceStart = rospy.Publisher(TOPIC_PEDIDOS_START, Int32, queue_size=5)
        self.subPedidosEnd = None
        self.subBotonPedidos = None
        self.pedidos_finished = False
        self.boton_presionado = False

    def execute(self, userdata):
        global estado_navegacion
        rospy.loginfo("Estado: VoiceState")
        self.pedidos_finished = False
        self.boton_presionado = False

        # Suscripcion a topics
        self.subPedidosEnd = rospy.Subscriber(TOPIC_PEDIDOS_END, Int32, self.pedidos_end_callback)
        self.subBotonPedidos = rospy.Subscriber(TOPIC_BOTON_PEDIDOS, Int32, self.boton_pedidos_callback)

        # Publicar en el topic de inicio voz
        self.pubVoiceStart.publish(1)

        rate = rospy.Rate(10)

        # Hasta que no termine pedidos o se pulse el botón de pedidos en el estado correspondiente
        while not (self.pedidos_finished or (self.boton_presionado and estado_navegacion == 2)):
            rate.sleep()

        # Desuscribirse después de terminar
        if self.subPedidosEnd:
            self.subPedidosEnd.unregister()
        if self.subBotonPedidos:
            self.subBotonPedidos.unregister()

        return 'voice_finished'

    # Callback para registrar el fin de los pedidos
    def pedidos_end_callback(self, msg):
        if msg.data == 1:  # Pedidos procesados
            self.pedidos_finished = True
            rospy.loginfo("Pedidos anotados")

    # Callback para registrar el boton de la interfaz
    def boton_pedidos_callback(self, msg):
        if msg.data == 1:  # Botón presionado
            self.boton_presionado = True
            rospy.loginfo("Pedidos leídos en cocina")


if __name__ == '__main__':
    rospy.init_node("fsm_robot")
    sm = StateMachine(outcomes=['end'])

    with sm:
        # Estado inicial
        StateMachine.add('ReposoState', ReposoState(), transitions={'task': 'NavigationState'})

        # Navegación
        StateMachine.add('NavigationState', NavigationState(), transitions={'to_voice': 'VoiceState', 'to_reposo': 'ReposoState'})

        # Estado voz 
        StateMachine.add('VoiceState', VoiceState(), transitions={'voice_finished': 'NavigationState'})

    # Iniciar introspección y ejecutar la máquina de estados
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()

