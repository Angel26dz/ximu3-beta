import time
import ximu3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import keyboard
 

beta = 0.1  
smoothed_acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float64)
SAMPLE_TIME = 0.02  

latest_quaternion = np.array([1, 0, 0, 0], dtype=np.float64)  
acceleration = np.array([0.0, 0.0, 0.0], dtype=np.float64)    
velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)
position = np.array([0.0, 0.0, 0.0], dtype=np.float64)

magnetometer_data = np.array([0.0, 0.0, 0.0])

fig_1 = plt.figure()
ax = fig_1.add_subplot(111, projection='3d')

fig_pos = plt.figure()
ax_pos = fig_pos.add_subplot(111, projection='3d')
coordenates=[]

fig_acc, ax_acc = plt.subplots()
time_data = []
acc_x_data = []
acc_y_data = []
acc_z_data = []

fig_vel, ax_vel = plt.subplots()
time_velocity = []
vel_x_data = []
vel_y_data = []
vel_z_data = []

origin = np.array([[0, 0, 0]]).T
X_vec = np.array([[1, 0, 0]]).T  
Y_vec = np.array([[0, 1, 0]]).T 
Z_vec = np.array([[0, 0, 1]]).T  

quiver_x = ax.quiver(origin[0], origin[1], origin[2], X_vec[0], X_vec[1], X_vec[2], color='r', length=1.0)
quiver_y = ax.quiver(origin[0], origin[1], origin[2], Y_vec[0], Y_vec[1], Y_vec[2], color='g', length=1.0)
quiver_z = ax.quiver(origin[0], origin[1], origin[2], Z_vec[0], Z_vec[1], Z_vec[2], color='b', length=1.0)

def quaternion_callback(message):
    return np.array([message.w, message.x, message.y, message.z], dtype=np.float64)

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x*y - w*z), 2 * (x*z + w*y)],
        [2 * (x*y + w*z), 1 - 2 * (x**2 + z**2), 2 * (y*z - w*x)],
        [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x**2 + y**2)]
    ], dtype=np.float64)

def acceleration_callback(message):
    g_to_m_s2 = 9.81  
    return np.array([
        message.accelerometer_x * g_to_m_s2, 
        message.accelerometer_y * g_to_m_s2, 
        (message.accelerometer_z - 1) * g_to_m_s2  
    ])


def magnetometer_callback(message):
    return np.array([message.x, message.y, message.z])

def q_callback(message):
    global latest_quaternion
    latest_quaternion = quaternion_callback(message)

def acc_callback(message):
    global acceleration, smoothed_acceleration

    acceleration = acceleration_callback(message)
    smoothed_acceleration = beta * acceleration + (1 - beta) * smoothed_acceleration 

    compute_velocity()  

def mag_callback(message):
    global mag
    mag = magnetometer_callback(message)
    

def compute_velocity():
    global velocity, smoothed_acceleration, latest_quaternion, SAMPLE_TIME, time_velocity
    global vel_x_data, vel_y_data, vel_z_data

    rotation_matrix = quaternion_to_rotation_matrix(latest_quaternion)


    acc_sensor = smoothed_acceleration * 9.81  

    acc_world = np.dot(rotation_matrix, acc_sensor)


    gravity_world = np.dot(rotation_matrix, np.array([0, 0, -9.81]))  
    acc_linear = acc_world - gravity_world 


    stationary_threshold = 10.3
    is_stationary = np.linalg.norm(acc_linear) < stationary_threshold

    if is_stationary:
        velocity = np.zeros(3) 
        #velocity *= 0.99    
    else:
        velocity += acc_linear * SAMPLE_TIME  

    velocity *= 0.99  

 
    time_velocity.append(time.time())
    vel_x_data.append(velocity[0])
    vel_y_data.append(velocity[1])
    vel_z_data.append(velocity[2])

    if len(time_velocity) > 100:
        time_velocity.pop(0)
        vel_x_data.pop(0)
        vel_y_data.pop(0)
        vel_z_data.pop(0)

    #print(f"Velocity: {velocity}")
    compute_position()


def compute_position():
    global position, velocity, SAMPLE_TIME
    global coordenates

     
    position += velocity * SAMPLE_TIME  
    position *= 0.98  # Atenuar el error acumulativo

      
    coordenates.append(position)

    

    print(f"Position: {position}")


def update_vel_plot(frame):
    ax_vel.clear()
    ax_vel.plot(time_velocity, vel_x_data, 'r-', label="Vel X")
    ax_vel.plot(time_velocity, vel_y_data, 'g-', label="Vel Y")
    ax_vel.plot(time_velocity, vel_z_data, 'b-', label="Vel Z")

    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")
    ax_vel.set_title("Real-Time Velocity Data")
    ax_vel.legend()
    ax_vel.relim()
    ax_vel.autoscale_view()


def update_plot(frame):
    global quiver_x, quiver_y, quiver_z

    quaternion = latest_quaternion.copy()
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)

    new_X = rotation_matrix @ X_vec
    new_Y = rotation_matrix @ Y_vec
    new_Z = rotation_matrix @ Z_vec

    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title("Real-Time IMU Orientation Tracking")

    quiver_x = ax.quiver(origin[0], origin[1], origin[2], new_X[0], new_X[1], new_X[2], color='r', length=1.0)
    quiver_y = ax.quiver(origin[0], origin[1], origin[2], new_Y[0], new_Y[1], new_Y[2], color='g', length=1.0)
    quiver_z = ax.quiver(origin[0], origin[1], origin[2], new_Z[0], new_Z[1], new_Z[2], color='b', length=1.0)

def update_acc_plot(frame):
    global time_data, acc_x_data, acc_y_data, acc_z_data

    time_data.append(time.time())
    acc_x_data.append(smoothed_acceleration[0])
    acc_y_data.append(smoothed_acceleration[1])
    acc_z_data.append(smoothed_acceleration[2])

    if len(time_data) > 100:  
        time_data.pop(0)
        acc_x_data.pop(0)
        acc_y_data.pop(0)
        acc_z_data.pop(0)

    ax_acc.clear()
    ax_acc.plot(time_data, acc_x_data, 'r-', label="Acc X")
    ax_acc.plot(time_data, acc_y_data, 'g-', label="Acc Y")
    ax_acc.plot(time_data, acc_z_data, 'b-', label="Acc Z")

    ax_acc.set_xlabel("Time (s)")
    ax_acc.set_ylabel("Acceleration (m/s²)")
    ax_acc.set_title("Real-Time Acceleration Data")
    ax_acc.legend()
    ax_acc.relim()
    ax_acc.autoscale_view()

def update_position_plot(frame):
    ax_pos.clear()

    if len(coordenates) > 0:
        coordenates_array = np.array(coordenates)  
        position_x_data = coordenates_array[:, 0]  
        position_y_data = coordenates_array[:, 1]  
        position_z_data = coordenates_array[:, 2]  

        ax_pos.plot(position_x_data, position_y_data, position_z_data, 'b-', marker="o", label="Trajectory")

    ax_pos.set_xlim(-50, 50)  
    ax_pos.set_ylim(-50, 50)  
    ax_pos.set_zlim(-50, 50) 

    ax_pos.set_xlabel("X Position (m)")
    ax_pos.set_ylabel("Y Position (m)")
    ax_pos.set_zlabel("Z Position (m)")
    ax_pos.set_title("Real-Time Position Tracking")
    ax_pos.legend()

    

def run():
    print("Searching for connections...")
    messages = ximu3.NetworkAnnouncement().get_messages_after_short_delay()

    if not messages:
        print("No TCP connections available. Using default IP and port.")
        connection_info = ximu3.TcpConnectionInfo("192.168.1.1", 7000)
    else:
        print(f"Found {messages[0].device_name} {messages[0].serial_number}")
        connection_info = messages[0].to_tcp_connection_info()

    connection = ximu3.Connection(connection_info)
    connection.add_quaternion_callback(q_callback)
    connection.add_inertial_callback(acc_callback)
    connection.add_magnetometer_callback(mag_callback)
    

    if connection.open() != ximu3.RESULT_OK:
        raise Exception("Unable to open connection")

    try:   
        print("Waiting 3 seconds before starting visualization...")
        time.sleep(3)

        #ani = animation.FuncAnimation(fig_1, update_plot, interval=100, cache_frame_data=False)
        #ani_acc = animation.FuncAnimation(fig_acc, update_acc_plot, interval=100, cache_frame_data=False)
        ani_vel = animation.FuncAnimation(fig_vel, update_vel_plot, interval=100, cache_frame_data=False)
        ani_pos = animation.FuncAnimation(fig_pos, update_position_plot, interval=300, cache_frame_data=False)

        while True:
            plt.pause(0.1)  
            if keyboard.is_pressed("esc"):  
                print("Stopping...")
                break  

    except KeyboardInterrupt:
        print("Interrupted by user.")

    finally:
        plt.close("all")  
        connection.close()
        print("Connection closed.")

run()