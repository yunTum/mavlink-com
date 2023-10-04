from pymavlink import mavutil

# Start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

while True:
    try:
        msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
        print(msg)
    except KeyboardInterrupt:
        print("diconnect")