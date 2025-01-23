#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Int32
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseActionResult

class TurtleBotNavigator:
    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)

        # Diccionario de posiciones:
        self.positions = {
            0: [2.0, -1.0, 0.0],
            1: [1.0, 0.0, 3.14],
            2: [1.0, -2.0, 3.14],
            3: [3.0, 0.0, 0.0],
            4: [3.0, -2.0, 0.0],
            10: [1, 4.0, 0.0]
        }

        # Suscriptores y publicadores
        rospy.Subscriber('/navegacion_start', Int32, self.navigation_callback)
        rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.result_callback)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.end_publisher = rospy.Publisher('/navegacion_end', Int32, queue_size=10)

        rate = rospy.Rate(10)
        rospy.loginfo("Nodo de navegacion iniciado")

        while 1:
            rate.sleep()
    
    # Función que envía una posición objetivo a alcanzar en función 
    # del valor recibido por el topic /navegacion_start
    def navigation_callback(self, msg):
        # Se envia un objetivo
        goal_id = msg.data
        if goal_id in self.positions:
            self.send_goal(goal_id)
        else:
            rospy.logwarn(f"ID recibida incorrecta: {goal_id}")

    # Funcion que envia un objetivo al turtlebot utilizando el stack de navegacion de ros
    def send_goal(self, goal_id):
        position = self.positions[goal_id]

        # Crear el mensaje PoseStamped
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.pose.position.x = position[0]
        goal_msg.pose.position.y = position[1]
        goal_msg.pose.position.z = 0
        quaternion = quaternion_from_euler(0, 0, position[2])
        goal_msg.pose.orientation.x = quaternion[0]
        goal_msg.pose.orientation.y = quaternion[1]
        goal_msg.pose.orientation.z = quaternion[2]
        goal_msg.pose.orientation.w = quaternion[3]

        # Publicar el objetivo
        self.goal_publisher.publish(goal_msg)
        rospy.loginfo(f"Enviando al robot a {position}")

    # Comprobar el resultado de la navegacion para saber cuando ha terminado
    def result_callback(self, msg):
        if msg.status.status == 3:  # Estado 3 significa que se alcanzó el objetivo
            rospy.loginfo("El TurtleBot ha alcanzado el objetivo.")
            self.end_publisher.publish(1)
        elif msg.status.status == 4:  # Estado 4 significa que falló la planificación
            rospy.logwarn("El TurtleBot no pudo alcanzar el objetivo.")

if __name__ == '__main__':
    try:
        navigator = TurtleBotNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
