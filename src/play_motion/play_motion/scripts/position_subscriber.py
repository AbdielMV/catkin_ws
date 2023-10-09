#!/usr/bin/env python

import rospy
import numpy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + ' I heard %s', data.data)
    print("La lista de Valores: \n")
    auxiliar_counter = 0
    union_list_position = []
    for index in range(16,len(data.position)):
        print(str(auxiliar_counter)+": \n")

        #Declaracion de variables auxiliares de la funcion
        union_name = data.name[index]
        union_position = data.position[index]

        #Conversion de Radianes a Grados
        union_position_in_degrees = From_Radian_to_Degrees(union_position)

        #Se imprimen los valores de la funcion y original
        print("A penas se va a insertar dentro del vector (radianes): "+str(union_position)+"\n")
        print("A penas se va a insertar dentro del vector (grados): "+str(union_position_in_degrees)+"\n")

        #Crear el vector con valores
        union_list_position.insert(auxiliar_counter,union_position)

        #Contador visual y local del 0 al 13
        auxiliar_counter = auxiliar_counter + 1

    #Poner un breakpoint para definir cuando termina la funcion for
    print("Termino la tarea de For \n")
    print("Lista final: "+str(union_list_position))

def From_Radian_to_Degrees(to_treatment_data):
    conversion_radian_degrees = (to_treatment_data*180)/math.pi
    print("Cuando se convierte de radianes a grados: "+str(conversion_radian_degrees)+"\n")
    return conversion_radian_degrees


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('joint_states', JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
