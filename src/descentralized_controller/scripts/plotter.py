#!/usr/bin/env python

import csv
import numpy as np
import matplotlib.pyplot as plt

# Function to read CSV file and extract data
def read_csv_data(filename):
    position = []
    velocity = []
    rhonn_position = []
    rhonn_velocity = []

    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            position.append(float(row[1]))
            velocity.append(float(row[2]))
            rhonn_position.append(float(row[3]))
            rhonn_velocity.append(float(row[4]))

    return position, velocity, rhonn_position, rhonn_velocity

# Function to generate a new time vector
def generate_time_vector(data_length, time_step=0.1):
    return np.arange(0, data_length * time_step, time_step)

# Function to plot data
def plot_data(time, position, velocity, rhonn_position, rhonn_velocity):
    plt.figure(figsize=(12, 6))

    # Plot position
    plt.subplot(2, 1, 1)
    plt.plot(time, position, label='Position')
    plt.plot(time,rhonn_position, label='Position_RHONN')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position vs. Time')
    plt.legend()

    # Plot velocity
    plt.subplot(2, 1, 2)
    plt.plot(time, velocity, label='Velocity', color='orange')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity vs. Time')
    plt.legend()

    plt.tight_layout()
    plt.show()

# Main function
if __name__ == '__main__':
    # Replace 'vector_data.csv' with your CSV file name
    filename = 'vector_data.csv'
    position, velocity, rhonn_position, rhonn_velocity = read_csv_data(filename)
    
    # Generate a new time vector
    time_step = 0.001  # Adjust this value as needed
    time = generate_time_vector(len(position), time_step)
    
    plot_data(time, position, velocity, rhonn_position, rhonn_velocity)
