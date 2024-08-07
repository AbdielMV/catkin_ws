#!/usr/bin/env python

import rospy
import csv
import numpy as np
import matplotlib.pyplot as plt

# Function to read CSV file and extract data
def read_csv_data(filename):
    position = []
    velocity = []
    rhonn_position = []
    rhonn_velocity = []
    rhonn_error_position = []
    rhonn_error_velocity = []
    set_point = []
    control_law = []
    error_tracking_x0 = []
    error_tracking_x1 = []
    surface = []
    w11 = []
    w12 = []
    w21 = []
    w22 = []

    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            position.append(float(row[1]))
            velocity.append(float(row[2]))
            rhonn_position.append(float(row[3]))
            rhonn_velocity.append(float(row[4]))
            rhonn_error_position.append(float(row[5]))
            rhonn_error_velocity.append(float(row[6]))
            set_point.append(float(row[7]))
            control_law.append(float(row[8]))
            error_tracking_x0.append([row[9]])
            error_tracking_x1.append([row[10]])
            surface.append(row[11])
            w11.append(row[12])
            w12.append(row[13])
            w21.append(row[14])
            w22.append(row[15])

    return position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point, control_law, error_tracking_x0, error_tracking_x1, surface, w11, w12, w21, w22

# Function to generate a new time vector
def generate_time_vector(data_length, time_step):
    return np.arange(0, data_length * time_step, time_step)

# Function to plot data
def plot_data(time, position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point, control_law, error_tracking_x0, error_tracking_x1, surface, w11, w12, w21, w22):
    plt.figure(1,figsize=(12, 6))

    # Plot error tracking position
    plt.subplot(2, 2, 1)
    plt.plot(time, surface, label='s')
    plt.xlabel('Seconds (s)')
    plt.ylabel('Surface')
    plt.title('Sliding Surface')
    plt.legend(loc='best')
    plt.grid()

    # Plot error x0
    plt.subplot(2, 2, 2)
    plt.plot(time,rhonn_error_position, label='Error_RHONN_x0')
    plt.xlabel('Seconds (s)')
    plt.ylabel('Degrees')
    plt.title('Error Identification Position')
    plt.legend(loc='best')
    plt.grid()

    # Plot control law
    plt.subplot(2, 2, 3)
    plt.plot(time, control_law, label='Control Law')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque')
    plt.title('Torque vs. Time')
    plt.legend(loc='best')
    plt.grid()

    # Plot error x1
    plt.subplot(2, 2, 4)
    plt.plot(time,rhonn_error_velocity, label='Error_RHONN_x1')
    plt.xlabel('Seconds (s)')
    plt.ylabel('Degrees/Seconds')
    plt.title('Error Identification Velocity')
    plt.legend(loc='best')
    plt.grid()

    plt.tight_layout()

    # Rhonn output
    plt.figure(2,figsize=(12, 6))

    plt.subplot(2,1,1)
    plt.plot(time, position, label='Position')
    plt.plot(time,rhonn_position, label='Position_RHONN')
    plt.plot(time, set_point, label = 'Reference')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time, velocity, label='Velocity')
    plt.plot(time, rhonn_velocity, label='Velocity_RHONN')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity vs. Time')
    plt.legend(loc='best')
    plt.grid()

    # Controller
    plt.figure(3,figsize=(12,6))

    plt.subplot(2,1,1)
    plt.plot(time, error_tracking_x0, label='Error Z0')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time, error_tracking_x1, label='Error Z1')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend(loc='best')
    plt.grid()

    # EFK Training
    plt.figure(4,figsize=(12,6))
    plt.subplot(2,1,1)
    plt.plot(time, w11, label='w11')
    plt.plot(time, w12, label='w12')
    plt.xlabel('Time (s)')
    plt.ylabel('EFK')
    plt.title('Weights Neuron 1') 
    plt.legend(loc='best')
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot(time, w21, label='w21')
    plt.plot(time, w22, label='w22')
    plt.xlabel('Time (s)')
    plt.ylabel('EFK')
    plt.title('Weights Neuron 2') 
    plt.legend(loc='best')
    plt.grid()

    


    plt.show()

# Main function
if __name__ == '__main__':
    # Replace 'vector_data.csv' with your CSV file name
    filename = 'vector_data.csv'
    position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point, control_law, error_tracking_x0, error_tracking_x1, surface, w11, w12, w21, w22 = read_csv_data(filename)
    
    # Generate a new time vector
    time_step = 0.005  # Adjust this value as needed(0.1)
    time = generate_time_vector(len(position), time_step)
    
    plot_data(time, position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point, control_law, error_tracking_x0, error_tracking_x1, surface, w11, w12, w21, w22)
