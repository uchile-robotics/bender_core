#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import mediapipe as mp
import math

class RaisedHand():
    def __init__(self):
        #Suscribir a camara
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

        self.image_pub = rospy.Publisher('/raised_hand/output_image', Image, queue_size=10)
        self.status_pub = rospy.Publisher('/raised_hand_status', String, queue_size=10)
        self.nose_pub = rospy.Publisher('/nose_position', Point, queue_size=10)


        # Inicializar los objetos de MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_draw = mp.solutions.drawing_utils 

        self.bridge = CvBridge()
    
    def calculate_angle(self, p1, p2,p3):
            
            #Calcular el ángulo entre tres puntos p1, p2, p3 (p2 es el vértice).
            #Devuelve el ángulo en grados.
        
            # Vectores desde p2 a p1 y desde p2 a p3
            vector1 = [p1.x - p2.x, p1.y - p2.y]
            vector2 = [p3.x - p2.x, p3.y - p2.y]

            # Producto punto y magnitudes de los vectores
            producto_punto = vector1[0] * vector2[0] + vector1[1] * vector2[1]
            magnitude1 = math.sqrt(vector1[0] ** 2 + vector1[1] ** 2)
            magnitude2 = math.sqrt(vector2[0] ** 2 + vector2[1] ** 2)

            # Ángulo en radianes
            cos_theta = producto_punto / (magnitude1 * magnitude2)
            angle = math.acos(cos_theta)  # Ángulo en radianes
            return math.degrees(angle)  # Conversión a grados
    
    def image_callback(self, msg):
        #Convertir mensaje de ROS a imagen OpenCV}
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding= 'bgr8')
        frame = cv2.flip(frame, 1)  # Voltear horizontalmente la imagen (espejo)

        # Convertir a RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Procesar la imagen con MediaPipe Hands
        hand_results = self.hands.process(rgb_frame)

        # Procesar la imagen con MediaPipe Pose
        pose_results = self.pose.process(rgb_frame)


        if hand_results.multi_hand_landmarks:  # Si hay manos detectadas
            for hand_landmarks in hand_results.multi_hand_landmarks:
                # Dibujar los puntos de la mano
                self.mp_draw.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # Obtener puntos clave de la mano
                wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                index_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]

                # Verificar si la mano está levantada
                is_hand_raised = index_finger_tip.y < wrist.y

                # Verificar si el brazo izquierdo está estirado
                if pose_results.pose_landmarks:  # Si hay puntos de pose detectados
                    left_elbow = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_ELBOW]
                    right_elbow = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
                    nose = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.NOSE]
        
                    is_left_arm_stretched = False
                    is_right_arm_stretched = False
                    # Verificar si el brazo izquierdo está estirado
                    if left_elbow:
                        shoulder = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
                        angle = self.calculate_angle(shoulder, left_elbow, wrist)
                        is_left_arm_stretched = angle > 160  # Umbral para determinar si el brazo está casi recto
                   
                    # Verificar si el brazo derecho está estirado
                    if right_elbow:
                        shoulder = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
                        angle = self.calculate_angle(shoulder, right_elbow, wrist)
                        is_right_arm_stretched = angle > 50  # Umbral para determinar si el brazo está casi recto

                if is_hand_raised and (is_left_arm_stretched or is_right_arm_stretched):
                    status_message = "True"
                    x=nose.x
                    y=nose.y
                    x_pixel = x * (640) #Multiplicamos por el ancho
                    y_pixel = y * (480) #Multiplicamos por el alto para que no este normalizado
                    
                    self.nose_pub.publish(Point(x=x_pixel, y=y_pixel, z=0))
                
                else:
                    status_message = "False"

                
                self.status_pub.publish(String(data=status_message))

                #Mostrar estado en la imagen
                cv2.putText(frame, f"Estado: {status_message}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                #Publicar imagen procesada
                output_image_msg = self.bridge.cv2_to_imgmsg(frame, encoding= 'bgr8')
                self.image_pub.publish(output_image_msg)



        # Mostrar la imagen procesada en una ventana
        cv2.imshow("Raised Hand Detection", frame)
        cv2.waitKey(1)



            
if __name__ == '__main__':
    rospy.init_node('raised_hand')
    raised_hand = RaisedHand()
    rospy.spin()