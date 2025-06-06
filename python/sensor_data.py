from pymavlink import mavutil
import time

# Connect to Pixhawk — adjust the serial port and baudrate as needed
mav = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
mav.wait_heartbeat()
print(f"Connected to system (System ID: {mav.target_system}, Component ID: {mav.target_component})")

# Request data streams from Pixhawk (helps if needed)
mav.mav.request_data_stream_send(
    mav.target_system,
    mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,  # Hz
    1    # Start streaming
)

# Print latest values in loop
while True:
    msg = mav.recv_match(blocking=True)

    if not msg:
        continue

    msg_type = msg.get_type()

    if msg_type == "ATTITUDE":
        print(f"[IMU] Roll: {msg.roll:.3f}, Pitch: {msg.pitch:.3f}, Yaw: {msg.yaw:.3f}")

    # elif msg_type == "RAW_IMU":
    #     print(f"[RAW_IMU] Accel: x={msg.xacc}, y={msg.yacc}, z={msg.zacc} | Gyro: x={msg.xgyro}, y={msg.ygyro}, z={msg.zgyro}")

    elif msg_type == "OPTICAL_FLOW_RAD":
        print(f"[Optical Flow] Flow_x: {msg.integrated_x:.4f}, Flow_y: {msg.integrated_y:.4f}, Distance: {msg.distance:.2f} m")

    elif msg_type == "DISTANCE_SENSOR":
        print(f"[LiDAR] Distance: {msg.current_distance / 100.0:.2f} m")

    elif msg_type == "LOCAL_POSITION_NED":
        print(f"[Local Position] x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    # Print 10 times per second
    time.sleep(0.1)
