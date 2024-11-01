import rclpy
from rclpy.node import Node
import numpy as np
from scipy.special import binom
from scipy.spatial.transform import Rotation as R

from crazychoir_interfaces.msg import FullState
from std_msgs.msg import Int32, Float32, Float64MultiArray as Array


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('states_processing')

        self.current_t = None
        self.t0 = None
        self.tf = None
        self.position = None
        self.velocity = None
        self.acceleration = None

        # Create subscribers
        self.pos_subscriber = self.create_subscription(Array, '/agent_0/position', self.pos_listener_callback, 10)
        self.vel_subscriber = self.create_subscription(Array, '/agent_0/velocity', self.vel_listener_callback, 10)
        self.acc_subscriber = self.create_subscription(Array, '/agent_0/acceleration', self.acc_listener_callback, 10)
        self.initial_time_subscriber = self.create_subscription(Int32, '/agent_0/t0', self.t0_listener_callback, 10)
        self.final_time_subscriber = self.create_subscription(Int32, '/agent_0/tf', self.tf_listener_callback, 10)
        self.current_time_subscriber = self.create_subscription(Float32, '/agent_0/current_t', self.t_listener_callback, 10)

        # Create publisher
        self.publisher_ = self.create_publisher(FullState, '/agent_0/fullstate', 1)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def pos_listener_callback(self, msg):
        try:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            data = [msg.data[i * cols:(i + 1) * cols] for i in range(rows)]
            self.position = np.array(data)
            self.get_logger().info(f'Position received: {self.position}')
        except Exception as e:
            self.get_logger().error(f'Error in position callback: {e}')

    def vel_listener_callback(self, msg):
        try:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            data = [msg.data[i * cols:(i + 1) * cols] for i in range(rows)]
            self.velocity = np.array(data)
            self.get_logger().info(f'Velocity received: {self.velocity}')
        except Exception as e:
            self.get_logger().error(f'Error in velocity callback: {e}')

    def acc_listener_callback(self, msg):
        try:
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            data = [msg.data[i * cols:(i + 1) * cols] for i in range(rows)]
            self.acceleration = np.array(data)
            self.get_logger().info(f'Acceleration received: {self.acceleration}')
        except Exception as e:
            self.get_logger().error(f'Error in acceleration callback: {e}')

    def tf_listener_callback(self, msg):
        self.tf = msg.data
        self.get_logger().info(f'TF received: {self.tf}')

    def t0_listener_callback(self, msg):
        self.t0 = msg.data
        self.get_logger().info(f'T0 received: {self.t0}')

    def t_listener_callback(self, msg):
        self.current_t = msg.data
        self.get_logger().info(f'Current time received: {self.current_t}')

    def publish_messages(self):
        if self.current_t is None or self.t0 is None or self.tf is None or self.position is None or self.velocity is None or self.acceleration is None:
            self.get_logger().warn('Waiting for all data to be available before publishing...')
            return

        # Ensure time is within bounds
        if not (self.t0 <= self.current_t <= self.tf):
            self.get_logger().warn(f'Current time {self.current_t} is out of bounds [{self.t0}, {self.tf}].')
            return

        try:
            # Calculate Bernstein basis
            N = 4
            time = self.current_t
            t0 = self.t0
            tf = self.tf
            cp = self.position
            cp_dot = self.velocity
            cp_ddot = self.acceleration

            b = np.zeros((N + 1, 1))
            for j in range(N + 1):
                b[j, 0] = binom(N, j) * ((time - t0) ** j * (tf - time) ** (N - j)) / (tf - t0) ** N

            ref = np.zeros(13)
            # Position
            ref[0] = cp[0] @ b
            ref[1] = cp[1] @ b
            ref[2] = cp[2] @ b

            # Attitude (assuming static attitude for this example)
            attitude_reference = np.zeros(3)  # Roll, Pitch, Yaw
            ref[3:7] = R.from_euler('xyz', attitude_reference).as_quat()

            # Velocity
            ref[7] = cp_dot[0] @ b
            ref[8] = cp_dot[1] @ b
            ref[9] = cp_dot[2] @ b

            # Acceleration
            ref[10] = cp_ddot[0] @ b
            ref[11] = cp_ddot[1] @ b
            ref[12] = cp_ddot[2] @ b

            # Publish full state message
            msg = FullState()
            msg.pose.position.x = ref[0]
            msg.pose.position.y = ref[1]
            msg.pose.position.z = ref[2]

            msg.pose.orientation.x = ref[3]
            msg.pose.orientation.y = ref[4]
            msg.pose.orientation.z = ref[5]
            msg.pose.orientation.w = ref[6]

            msg.vel.linear.x = ref[7]
            msg.vel.linear.y = ref[8]
            msg.vel.linear.z = ref[9]

            msg.acc.linear.x = ref[10]
            msg.acc.linear.y = ref[11]
            msg.acc.linear.z = ref[12]

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published full state: {ref}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_messages: {e}')


def main(args=None):
    rclpy.init(args=args)
    traj_publisher = TrajectoryPublisher()
    rclpy.spin(traj_publisher)
    traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
