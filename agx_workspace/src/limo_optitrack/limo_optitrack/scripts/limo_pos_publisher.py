#!/usr/bin/env python3
from limo_optitrack.scripts.OptiTrackPython import NatNetClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from threading import Lock


class Optitrack(object):

    def __init__(
        self,
        client_ip="192.168.0.104",  # change it to your LIMO IP address
        server_ip="192.168.0.100",  # change it to your OPTITRACK PC IP address
        multicast_address="239.255.42.99",  # change it to your multicast address
        rigidbody_names2track=["limo1336"],  # change it to your limo name on the OPTITRACK system
    ):
        if client_ip is None:
            hostname = socket.gethostname()
            client_ip = socket.gethostbyname(hostname)
        print(f"client ip: {client_ip}")
        print(f"server ip: {server_ip}")
        print(f"multicast address: {multicast_address}")
        print(f"body names to track: {rigidbody_names2track}")

        self.rigidbody_names2track = rigidbody_names2track
        self.lock_opti = Lock()
        self.optitrack_reading = {}
        # This will create a new NatNet client
        self.streamingClient = NatNetClient(client_ip, server_ip, multicast_address)

        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streamingClient.rigidBodyListener = self.receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        self.streamingClient.run()
        self.read_new_data()
        self.limo_position = np.full((2,), np.nan)

    def close(self):
        self.streamingClient.close()

    def read_new_data(self):
        for key in self.optitrack_reading:
            state_data = self.optitrack_reading[key][1]
            for ii in range(2):
                self.limo_position[ii] = np.array(state_data[ii], dtype=float)

    def receive_rigid_body_frame(
        self, timestamp, id, position, rotation, rigidBodyDescriptor
    ):
        if rigidBodyDescriptor:
            for rbname in self.rigidbody_names2track:
                if rbname in rigidBodyDescriptor:
                    if id == rigidBodyDescriptor[rbname][0]:
                        # skips this message if still locked
                        if self.lock_opti.acquire(False):
                            try:
                                # rotation is a quaternion!
                                self.optitrack_reading[rbname] = [
                                    timestamp,
                                    position,
                                    rotation,
                                ]
                            finally:
                                self.lock_opti.release()


class LimoPosPublisher(Node):
    def __init__(self):
        super().__init__('limo_1336_pos_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, 'limo_1336_pos', 10)
        timer_period = 0.5  # publishing duration
        self.practitioner = Optitrack()
        self.limo_position = np.array([np.nan, np.nan])  # set up the first position to nan to avoid the wrong position
        # publishing
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64MultiArray()
        self.practitioner.read_new_data()
        self.limo_position = self.practitioner.limo_position
        if np.isnan(self.limo_position).any():
            self.get_logger().info("Optitrack is not working, waiting...")
        else:
            msg.data = self.limo_position.tolist()  # Convert numpy array to list
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing limo 1336 pos: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    limos_pos_publisher = LimoPosPublisher()
    rclpy.spin(limos_pos_publisher)
    limos_pos_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
