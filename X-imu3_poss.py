You said:
import time

import ximu3

import numpy as np

import keyboard

import matplotlib.pyplot as plt

import matplotlib.animation as animation

from collections import deque



beta = 0.1  

SAMPLE_TIME = 0.02  





REST_THRESHOLD = 0.15  





time_window = 100

time_series = deque(maxlen=time_window)

acc_x_series = deque(maxlen=time_window)

acc_y_series = deque(maxlen=time_window)

acc_z_series = deque(maxlen=time_window)

vel_x_series = deque(maxlen=time_window)

vel_y_series = deque(maxlen=time_window)

vel_z_series = deque(maxlen=time_window)



velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)

smoothed_acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float64)

latest_quaternion = np.array([1, 0, 0, 0], dtype=np.float64)

initial_quaternion = None  

stationary_time = 0  



gravity_vector = np.array([0, 0, -9.81])  



fig_acc, ax_acc = plt.subplots()

fig_vel, ax_vel = plt.subplots()



lines_acc = [ax_acc.plot([], [], label=label)[0] for label in ['Acc X', 'Acc Y', 'Acc Z']]

lines_vel = [ax_vel.plot([], [], label=label)[0] for label in ['Vel X', 'Vel Y', 'Vel Z']]



ax_acc.legend()

ax_acc.set_ylim(-10, 10)

ax_acc.set_xlabel("Time")

ax_acc.set_ylabel("Acceleration (m/s²)")



ax_vel.legend()

ax_vel.set_ylim(-2, 2)

ax_vel.set_xlabel("Time")

ax_vel.set_ylabel("Velocity (m/s)")



def quaternion_callback(message):

    return np.array([message.w, message.x, message.y, message.z], dtype=np.float64)



def quaternion_to_rotation_matrix(q):



    w, x, y, z = q

    return np.array([

        [1 - 2 * (y**2 + z**2), 2 * (x*y - w*z), 2 * (x*z + w*y)],

        [2 * (x*y + w*z), 1 - 2 * (x**2 + z**2), 2 * (y*z - w*x)],

        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x**2 + y**2)]

    ], dtype=np.float64)



def quaternion_conjugate(q):



    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)



def quaternion_multiply(q1, q2):



    w1, x1, y1, z1 = q1

    w2, x2, y2, z2 = q2

    return np.array([

        w1*w2 - x1*x2 - y1*y2 - z1*z2,

        w1*x2 + x1*w2 + y1*z2 - z1*y2,

        w1*y2 - x1*z2 + y1*w2 + z1*x2,

        w1*z2 + x1*y2 - y1*x2 + z1*w2

    ], dtype=np.float64)



def adjust_to_reference(q):

    global initial_quaternion

    if initial_quaternion is None:

        initial_quaternion = q  

    q_relative = quaternion_multiply(quaternion_conjugate(initial_quaternion), q)

    return q_relative





def g_callback(message):

    return np.radians(np.array([

        message.gyroscope_x, 

        message.gyroscope_y, 

        message.gyroscope_z

    ], dtype=np.float64))



def acceleration_callback(message):

    return np.array([

        message.accelerometer_x * 9.81, 

        message.accelerometer_y * 9.81, 

        message.accelerometer_z * 9.81  

    ], dtype=np.float64)



def q_callback(message):

    global latest_quaternion

    latest_quaternion = adjust_to_reference(quaternion_callback(message))





def compute_velocity(acceleration):

   

    global velocity, stationary_time



    alpha = 0.95  

    linear_acceleration = alpha * (acceleration - gravity_vector) + (1 - alpha) * acceleration





    filtered_acc_norm = np.linalg.norm(linear_acceleration)



    is_stationary = filtered_acc_norm < REST_THRESHOLD



    if is_stationary:

        stationary_time += SAMPLE_TIME

        if stationary_time > 3:  

            velocity = np.zeros(3)  

        else:

            velocity *= 0.95  

    else:

        stationary_time = 0  

        velocity += linear_acceleration * SAMPLE_TIME  



    velocity *= 0.99 

    return velocity



def acc_callback(message):



    global smoothed_acceleration





    acceleration = acceleration_callback(message)

    smoothed_acceleration = beta * acceleration + (1 - beta) * smoothed_acceleration

    rotation_matrix = quaternion_to_rotation_matrix(latest_quaternion)

    acceleration_global = rotation_matrix @ smoothed_acceleration



    linear_acceleration = acceleration_global - gravity_vector



    velocity = compute_velocity(linear_acceleration)





    time_series.append(time.time())

    acc_x_series.append(linear_acceleration[0])

    acc_y_series.append(linear_acceleration[1])

    acc_z_series.append(linear_acceleration[2])

    vel_x_series.append(velocity[0])

    vel_y_series.append(velocity[1])

    vel_z_series.append(velocity[2])



    print(f'Velocity: {velocity}')



def update_acc_plot(frame):

    for line, data in zip(lines_acc, [acc_x_series, acc_y_series, acc_z_series]):

        line.set_xdata(list(time_series))

        line.set_ydata(list(data))

    ax_acc.relim()

    ax_acc.autoscale_view()

    return lines_acc



def update_vel_plot(frame):

    for line, data in zip(lines_vel, [vel_x_series, vel_y_series, vel_z_series]):

        line.set_xdata(list(time_series))

        line.set_ydata(list(data))

    ax_vel.relim()

    ax_vel.autoscale_view()

    return lines_vel



def main():

    print("Searching for connections...")

    messages = ximu3.NetworkAnnouncement().get_messages_after_short_delay()

    connection_info = messages[0].to_tcp_connection_info() if messages else ximu3.TcpConnectionInfo("192.168.1.1", 7000)



    connection = ximu3.Connection(connection_info)

    connection.add_quaternion_callback(q_callback)

    connection.add_inertial_callback(acc_callback)

    connection.add_inertial_callback(g_callback)



    if connection.open() != ximu3.RESULT_OK:

        raise Exception("Unable to open connection")



    try:

        print("Waiting 3 seconds before starting visualization...")

        time.sleep(3)

        ani_acc = animation.FuncAnimation(fig_acc, update_acc_plot, interval=100, cache_frame_data=False)

        ani_vel = animation.FuncAnimation(fig_vel, update_vel_plot, interval=100, cache_frame_data=False)



        while True:

            time.sleep(0.1)

            if keyboard.is_pressed("esc"):

                print("Stopping...")

                break  



    except KeyboardInterrupt:

        print("Interrupted by user.")



    finally:

        connection.close()
        plt.close('all')
        print("Connection closed.")



if __name__ == '__main__':

    main()
