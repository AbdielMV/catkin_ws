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

    return position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point

# Function to generate a new time vector
def generate_time_vector(data_length, time_step):
    return np.arange(0, data_length * time_step, time_step)

# Function to plot data
def plot_data(time, position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point):
    plt.figure(figsize=(12, 6))

    # Plot position
    plt.subplot(2, 2, 1)
    plt.plot(time, position, label='Position')
    plt.plot(time,rhonn_position, label='Position_RHONN')
    plt.plot(time, set_point, label = 'Reference')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend()
    plt.grid()

    # Plot error x0
    plt.subplot(2, 2, 2)
    plt.plot(time,rhonn_error_position, label='Error_RHONN_x0')
    plt.xlabel('Seconds (s)')
    plt.ylabel('Degrees')
    plt.title('Error Identification Position')
    plt.legend()
    plt.grid()

    # Plot velocity
    plt.subplot(2, 2, 3)
    plt.plot(time, velocity, label='Velocity', color='orange')
    plt.plot(time, rhonn_velocity, label='Velocity_RHONN')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity vs. Time')
    plt.legend()
    plt.grid()

    # Plot error x1
    plt.subplot(2, 2, 4)
    plt.plot(time,rhonn_error_velocity, label='Error_RHONN_x1')
    plt.xlabel('Seconds (s)')
    plt.ylabel('Degrees/Seconds')
    plt.title('Error Identification Velocity')
    plt.legend()
    plt.grid()

    # # New Figure
    # plt.figure(figsize=(12, 6))

    # # Plot position
    # plt.subplot(2, 2, 1)
    # plt.plot(time, position, label='Position')
    # plt.plot(time,rhonn_position, label='Position_RHONN')
    # plt.plot(time, set_point, label = 'Reference')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Position')
    # plt.title('Position vs. Time')
    # plt.legend()
    # plt.grid()

    # # Plot error x0
    # plt.subplot(2, 2, 2)
    # plt.plot(time,rhonn_error_position, label='Error_RHONN_x0')
    # plt.xlabel('Seconds (s)')
    # plt.ylabel('Degrees')
    # plt.title('Error Identification Position')
    # plt.legend()
    # plt.grid()

    # # Plot velocity
    # plt.subplot(2, 2, 3)
    # plt.plot(time, velocity, label='Velocity', color='orange')
    # plt.plot(time, rhonn_velocity, label='Velocity_RHONN')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Velocity')
    # plt.title('Velocity vs. Time')
    # plt.legend()
    # plt.grid()

    # # Plot error x1
    # plt.subplot(2, 2, 4)
    # plt.plot(time,rhonn_error_velocity, label='Error_RHONN_x1')
    # plt.xlabel('Seconds (s)')
    # plt.ylabel('Degrees/Seconds')
    # plt.title('Error Identification Velocity')
    # plt.legend()
    # plt.grid()

    plt.tight_layout()
    plt.show()

# Main function
if __name__ == '__main__':
    # Replace 'vector_data.csv' with your CSV file name
    filename = 'vector_data.csv'
    position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point = read_csv_data(filename)
    
    # Generate a new time vector
    time_step = 0.01  # Adjust this value as needed
    time = generate_time_vector(len(position), time_step)
    
    plot_data(time, position, velocity, rhonn_position, rhonn_velocity, rhonn_error_position, rhonn_error_velocity, set_point)
