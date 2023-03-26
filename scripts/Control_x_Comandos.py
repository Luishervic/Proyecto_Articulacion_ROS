#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import JointState
import serial

ser = serial.Serial(
        # Puerto serie del que leer los datos
        port='/dev/ttyACM0',
 
        # Velocidad de transmisión de datos
        baudrate = 115200,
   
        # Paridad (ninguna en este caso)
        parity=serial.PARITY_NONE,
 
        # Patrón de bits de parada
        stopbits=serial.STOPBITS_ONE,
     
        # Número total de bits
        bytesize=serial.EIGHTBITS,
 
        # Número de comandos serie a aceptar antes de expirar
        timeout=2
)

def main():
    rospy.init_node("joint_state_publisher")
    pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Leer datos desde el puerto serie
        x = ser.readline().decode('utf-8') # Convierte los bytes a una cadena de texto
        values = x.strip().split("\t") # Elimina los caracteres especiales de la cadena de texto       
        try:
            valor1 = float(values[0])  # Convierte el valor a float
            valor2 = float(values[1])  # Convierte el valor a float
            valor3 = float(values[2])  # Convierte el valor a float
        except (ValueError,IndexError):
            continue
        
        # Crear mensaje de estado 
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = ["Revolute_2"]
        joint_state_msg.position = [round(valor3/100,3)]  # Publica posición
        joint_state_msg.velocity = [valor2]  # Publica velocidad
        joint_state_msg.effort = [valor1]  # PUblica setpoint
        
        
        # Publicar mensaje de estado conjunto
        pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()      
    except rospy.ROSInterruptException:
        pass
