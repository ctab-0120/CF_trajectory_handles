import rclpy
from rclpy.node import Node
import numpy as np
from scipy.special import binom

from std_msgs.msg import Int32, Float32, MultiArrayDimension, MultiArrayLayout, Float64MultiArray as Array


class ControlPointsPublisher(Node):

    def __init__(self):
        # Create the control_points node
        super().__init__('control_points')
        
        # Initialize the topics
        self.pos_publisher = self.create_publisher(Array, 'agent_0/position', 1)
        self.vel_publisher = self.create_publisher(Array, 'agent_0/velocity', 1)
        self.acc_publisher = self.create_publisher(Array, 'agent_0/acceleration', 1)
        self.initial_time_publisher = self.create_publisher(Int32, 'agent_0/t0', 1)
        self.final_time_publisher = self.create_publisher(Int32, 'agent_0/tf', 1)
        self.time_publisher = self.create_publisher(Float32, 'agent_0/current_t', 1)
        
        # Get ROS time at t=0
        self.start_time_sec = self.get_time()
        
        # Initialize the timer
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def publish_messages(self):
        
        # Compute time t
        time = self.get_time() - self.start_time_sec
        current_t = Float32()
        current_t.data = time
        self.time_publisher.publish(current_t)
            
        N = 4
        if 0 < time < 20:
            t0 = 0
            tf = 40
            
            control_p =np.array([[0.0, 1.0, 2.0, 3.0, 4.0],
                          [0.0, 1.0, 2.0, 3.0, 4.0],
                          [1.0, 1.0, 1.0, 1.0, 1.0]])
        else:
            t0 = 20
            tf = 60
            control_p =np.array([[2.0, 2.5, 3.0, 5.0, 6.0],
                                 [2.0, 0.0, 1.0, 3.0, 4.5],
                                 [1.0, 1.0, 1.0, 1.0, 1.0]])
        
        # Build the differentiation matrix
        Dm = diffMatrix(N, t0, tf)
        
        cp = Array()
        cp.layout.dim.append(MultiArrayDimension(label='row', size=3, stride=3*(N+1)))
        cp.layout.dim.append(MultiArrayDimension(label='col', size=N+1, stride=N+1))
        cp.layout.data_offset = 0
        cp.data = control_p.flatten().tolist()
        
        self.pos_publisher.publish(cp)
        
        cp_dot = Array()
        cp_dot.layout.dim.append(MultiArrayDimension(label='row', size=3, stride=3*(N+1)))
        cp_dot.layout.dim.append(MultiArrayDimension(label='col', size=N+1, stride=N+1))
        cp_dot.layout.data_offset = 0
        cp_dot_temp = control_p @ Dm
        cp_dot.data = cp_dot_temp.flatten().tolist()
        
        self.vel_publisher.publish(cp_dot)
        
        cp_ddot = Array()
        cp_ddot.layout.dim.append(MultiArrayDimension(label='row', size=3, stride=3*(N+1)))
        cp_ddot.layout.dim.append(MultiArrayDimension(label='col', size=N+1, stride=N+1))
        cp_ddot.layout.data_offset = 0
        cp_ddot_temp = cp_dot_temp @ Dm
        cp_ddot.data = cp_ddot_temp.flatten().tolist()
        
        self.acc_publisher.publish(cp_ddot)
        
        final_t = Int32()
        final_t.data = tf
        
        self.final_time_publisher.publish(final_t)
        
        initial_t = Int32()
        initial_t.data = t0
        
        self.initial_time_publisher.publish(initial_t)
        
        self.get_logger().info('Publishing')
        
    def get_time(self):
        sec = self.get_clock().now().to_msg().sec
        nsec = self.get_clock().now().to_msg().nanosec/1e9
        return sec + nsec
    
def diffMatrix(n, t0=0.0, tf=1.0):
    # Differentiation matrix from BeBOT
    val = n / (tf - t0)
    temp = np.zeros((n+1, n))
    for i in range(n):
        temp[i, i] = -val
        temp[i+1, i] = val
        
    # Elevate to make it square
    T = elevMatrix(n-1, 1)
    Dm = temp @ T

    return Dm
    
def elevMatrix(N, R):
    # Elevation matrix from BeBOT
    T = np.zeros((N+1, N+R+1))
    for i in range(N+R+1):
        den = binom(N+R, i)
        for j in range(N+1):
            T[j, i] = binom(N, j) * binom(R, i-j) / den

    return T
        
def main(args=None):
    rclpy.init(args=args)

    traj_publisher = ControlPointsPublisher()

    rclpy.spin(traj_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

