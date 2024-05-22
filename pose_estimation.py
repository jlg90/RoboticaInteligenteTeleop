#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
from ackermann_msgs.msg import AckermannDrive

# Declarar el detector de pose de MediaPipe a utilizar
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=2)
mp_drawing = mp.solutions.drawing_utils

# Publicador de mensajes de control
ackermann_command_publisher = None

# Función para determinar si la mano está cerrada
def is_hand_closed(landmarks):
    tips = [landmarks[8], landmarks[12], landmarks[16], landmarks[20]]
    base = landmarks[9]
    forward = landmarks[4]
    back = landmarks[17]
    if all(tip.y > base.y for tip in tips):
        if forward.x > back.x:
            poseMov = "moverAtras"
        else:
            poseMov = "moverAlante"
    else:
        poseMov = "quieto"
            
    
    return poseMov

# Función para determinar si hay un dedo levantado
def rotation(landmarks):
    finger = landmarks[4]
    base = landmarks[17]
    
    return finger.x < base.x

# Función para determinar si ambas manos están abiertas
def both_hands_open(landmarks1, landmarks2):
    return not is_hand_closed(landmarks1) and not is_hand_closed(landmarks2)

# Procesar la imagen del operador
def image_callback(msg):
    global ackermann_command_publisher
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # Procesar la imagen con MediaPipe
    image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = hands.process(image_rgb)

    gesture_right = None
    gesture_left = None
    reverse = False

    if results.multi_handedness and results.multi_hand_landmarks:
        for idx, handedness in enumerate(results.multi_handedness):
            landmarks = results.multi_hand_landmarks[idx]
            label = handedness.classification[0].label

            if label == 'Right':  # Mano derecha
                if is_hand_closed([lm for lm in landmarks.landmark]) == "moverAlante":
                    gesture_right = "go"
                    color = (0, 255, 0)
                elif is_hand_closed([lm for lm in landmarks.landmark]) == "moverAtras":
                    gesture_right = "reverse"
                    color = (0, 255, 255) 
                else:
                    gesture_right = "stop"
                    color = (0, 0, 255)  # Rojo para mano abierta
            elif label == 'Left':  # Mano izquierda
                if not rotation([lm for lm in landmarks.landmark]):
                    gesture_left = "turn_right"
                    color = (255, 0, 0)  # Azul para un dedo levantado
                elif rotation([lm for lm in landmarks.landmark]):
                    gesture_left = "turn_left"
                    color = (255, 255, 0)  # Amarillo para dos dedos levantados
                elif is_hand_closed([lm for lm in landmarks.landmark]):
                    gesture_left = "none"
                    color = (128, 128, 128)  # Gris para mano cerrada

            # Dibujar los landmarks sobre la imagen con el color adecuado
            mp_drawing.draw_landmarks(
                cv_image,
                landmarks,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing.DrawingSpec(color=color, thickness=2, circle_radius=2),
                mp_drawing.DrawingSpec(color=color, thickness=2, circle_radius=2)
            )

        # Comprobar si ambas manos están abiertas para retroceder
        if len(results.multi_hand_landmarks) == 2:
            if both_hands_open([lm for lm in results.multi_hand_landmarks[0].landmark], [lm for lm in results.multi_hand_landmarks[1].landmark]):
                reverse = True

        # Publicar el comando de control basado en las manos
        command = AckermannDrive()
        if reverse:
            command.speed = -1.0
            command.steering_angle = 0.0
        else:
            if gesture_right == "go":
                command.speed = 1.0
                command.steering_angle = 0.0
            elif gesture_right == "stop":
                command.speed = 0.0
                command.steering_angle = 0.0
            elif gesture_right == "reverse":
                command.speed = -0.5
                command.steering_angle = 0.0

            if gesture_left == "turn_right":
                command.steering_angle = -0.5  # Girar a la derecha
            elif gesture_left == "turn_left":
                command.steering_angle = 0.5  # Girar a la izquierda

        ackermann_command_publisher.publish(command)

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)

def main():
    global ackermann_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    # Definición del publicador
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
