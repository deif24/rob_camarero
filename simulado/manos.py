#!/usr/bin/env python3
import rospy
import cv2
import mediapipe as mp
from std_msgs.msg import Int32
import time 

# Configurar mediapipe
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils
pose = mp_pose.Pose()

class ManoDetectada:
    def __init__(self):
        rospy.init_node('deteccion_manos', anonymous=True)

        # Suscripción y publicación de topics
        self.start_subscriber = rospy.Subscriber('/manos_start', Int32, self.start_callback)
        self.end_publisher = rospy.Publisher('/manos_end', Int32, queue_size=10)
        self.deteccion_activada = False

        # Variables de tiempo
        self.start_time = None
        self.mano_levantada = False
        
        rospy.loginfo("Nodo de detección de manos iniciado")

    # Ejecución del programa cuando se recibe un "1" en el topic de inicio
    def start_callback(self, msg):
        if msg.data == 1:
            self.deteccion_activada = True
            rospy.loginfo("Inicio deteccion de manos")
            self.detectar_manos()

    def detectar_manos(self):
        cap = cv2.VideoCapture(0)
        while self.deteccion_activada:
            ret, frame = cap.read()
            if not ret:
                rospy.logwarn("No se ha podido leer la cámara")
                break

            # Procesar la imagen
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False
            results = pose.process(image)
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Si se han obtenido landmarks
            if results.pose_landmarks:
                try:
                    """
                    # Dibujar keypoints
                    mp_drawing.draw_landmarks(
                        image,                          
                        results.pose_landmarks,         
                        mp_pose.POSE_CONNECTIONS,   
                        mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),  
                        mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)                   
                    )
                    """

                    # Obtener posiciones clave del brazo derecho e izquierdo (muñeca, codo, hombro)
                    left_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_WRIST]
                    right_wrist = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_WRIST]
                    left_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_ELBOW]
                    right_elbow = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_ELBOW]
                    left_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
                    right_shoulder = results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]

                    # Comprobar si las muñecas y codos están por encima de los hombros
                    left_hand_up = left_wrist.y < left_shoulder.y and left_elbow.y < left_shoulder.y
                    right_hand_up = right_wrist.y < right_shoulder.y and right_elbow.y < right_shoulder.y

                    # Verificar si una de las manos está levantada
                    if left_hand_up or right_hand_up:
                        if not self.mano_levantada:  # Si la mano no estaba levantada previamente
                            self.mano_levantada = True
                            self.start_time = time.time()  # Iniciar temporizador
                        
                        # Comprobar si la mano lleva levantada más de 1 segundo
                        if time.time() - self.start_time >= 1:
                            self.end_publisher.publish(1)
                            self.deteccion_activada = False
                            rospy.loginfo("Mano levantada detectada durante más de 1 segundo")
                            break
                    else:
                        # Si las manos no están levantadas, reiniciar el estado
                        self.mano_levantada = False
                        self.start_time = None

                except IndexError:
                    rospy.logwarn("No se han podido obtener landmarks correctamente")

            # Mostrar la imagen (opcional, puede desactivarse para mejorar rendimiento)
            cv2.imshow("Pose Detection", image)

            # Salir si se presiona 'q'
            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

        # Liberar la cámara y recursos
        cap.release()
        cv2.destroyAllWindows()

        # Esperar un nuevo mensaje
        self.deteccion_activada = False
        rospy.loginfo("Esperando nuevo mensaje para comenzar la detección.")

if __name__ == '__main__':
    try:
        detector = ManoDetectada()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
