#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
import serial

ser = serial.Serial(
    # Puerto serie al que enviar los datos
    port='/dev/ttyACM0',
 
    # Velocidad de transmisión de datos
    baudrate=115200,
   
    # Paridad (ninguna en este caso)
    parity=serial.PARITY_NONE,
 
    # Patrón de bits de parada
    stopbits=serial.STOPBITS_ONE,
     
    # Número total de bits
    bytesize=serial.EIGHTBITS,
 
    # Número de comandos serie a aceptar antes de expirar
    timeout=2
)

def joint_state_callback(msg):
    if len(msg.position) > 0:
        joint_pos = 'M' + str(round(msg.position[0]*100,3))  # Obtener posición del primer joint      
        # Enviar posición por el puerto serie
        print(joint_pos)
        ser.write(joint_pos.encode('utf-8') + b'\n')

def main():
    rospy.init_node("joint_state_subscriber")
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()      
    except rospy.ROSInterruptException:
        pass

