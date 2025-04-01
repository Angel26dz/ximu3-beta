import time
import ximu3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import keyboard
 

samplePeriod = 1/256
g_to_mps2 =9.81

acc_vector= np.array([0.0,0.0,0.0],dtype=np.float64)
gyr_vector= np.array([0.0,0.0,0.0],dtype=np.float64)
mag_vector= np.array([0.0,0.0,0.0],dtype=np.float64)



time_data = []  
acc_x_data = []
acc_y_data = []
acc_z_data = []

def acceleration_callback(message):
    global g_to_mps2 
    acc_x = message.acceleration_x * g_to_mps2
    acc_y = message.acceleration_y * g_to_mps2
    acc_z = message.acceleration_z * g_to_mps2
    return acc_x, acc_y, acc_z
def gyroscope_callback(message):
    return message.gyroscope_x, message.gyroscope_y, message.gyroscope_z,


def magnetometer_callback(message):
    return message.x, message.y, message.z

def get_acc_(message):
    global time_data, acc_x_data, acc_y_data, acc_z_data
    
    acc_x, acc_y, acc_z = acceleration_callback(message)


    time_data.append(time.time()) 
    acc_x_data.append(acc_x)
    acc_y_data.append(acc_y)
    acc_z_data.append(acc_z)

    if len(time_data) > 100:
        time_data.pop(0)
        acc_x_data.pop(0)
        acc_y_data.pop(0)
        acc_z_data.pop(0)

def main():
    print("Searching for connections...")
    messages = ximu3.NetworkAnnouncement().get_messages_after_short_delay()

    if not messages:
        print("No TCP connections available. Using default IP and port.")
        connection_info = ximu3.TcpConnectionInfo("192.168.1.1", 7000)
    else:
        print(f"Found {messages[0].device_name} {messages[0].serial_number}")
        connection_info = messages[0].to_tcp_connection_info()

    connection = ximu3.Connection(connection_info)
    connection.add_magnetometer_callback(magnetometer_callback)
    connection.add_linear_acceleration_callback(acceleration_callback)

    try:   

        print('asd')


        while True:
            plt.pause(0.1)  
            if keyboard.is_pressed("esc"):  
                print("Stopping...")
                break  

    except KeyboardInterrupt:
        print("Interrupted by user.")

if __name__ == '__main__':
    main()
