import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint
from nav_msgs.msg import Odometry
import csv
import os

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('states_processing')

        self.N = 16  # Number of robots

        # Initialize position attributes for each robot
        self.positions = {i: {'x': None, 'y': None, 'z': None} for i in range(self.N)}
        self.odometry_data = {i: {'position': None, 'orientation': None, 'linear_velocity': None, 'angular_velocity': None} for i in range(self.N)}

        # Create subscribers and publishers for each robot
        self.pos_subscribers = {}
        self.publishers_traj_params = {}
        self.odom_subscribers = {}
        
        for i in range(self.N):
            # Position subscribers
            self.pos_subscribers[i] = self.create_subscription(
                Point, f'/desired_position_{i}', 
                lambda msg, idx=i: self.pos_listener_callback(msg, idx), 
                10
            )
            # Trajectory publishers
            self.publishers_traj_params[i] = self.create_publisher(
                Trajectory, f'/agent_{i}/traj_params', 1
            )
            # Odometry subscribers
            self.odom_subscribers[i] = self.create_subscription(
                Odometry, f'/agent_{i}/odom', 
                lambda msg, idx=i: self.odom_listener_callback(msg, idx), 
                10
            )

        # Set setpoint height for all robots
        self.height = 1  # meter

        # Create a timer to publish messages periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

        # Create a directory for storing CSV files if it doesn't exist
        self.csv_directory = 'odometry_data'
        os.makedirs(self.csv_directory, exist_ok=True)

        # Create a CSV file and write the header
        self.csv_filename = os.path.join(self.csv_directory, 'odometry_data.csv')
        self.create_csv_file()

    def create_csv_file(self):
        """Create a new CSV file and write the header."""
        try:
            with open(self.csv_filename, mode='w', newline='') as csvfile:
                fieldnames = ['time_stamp']
                # Add field names for each robot
                for i in range(self.N):
                    fieldnames.extend([
                        f'robot_{i}_position_x', f'robot_{i}_position_y', f'robot_{i}_position_z', 
                        f'robot_{i}_orientation_x', f'robot_{i}_orientation_y', f'robot_{i}_orientation_z', f'robot_{i}_orientation_w',
                        f'robot_{i}_linear_velocity_x', f'robot_{i}_linear_velocity_y', f'robot_{i}_linear_velocity_z',
                        f'robot_{i}_angular_velocity_x', f'robot_{i}_angular_velocity_y', f'robot_{i}_angular_velocity_z'
                    ])
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
        except Exception as e:
            self.get_logger().error(f'Error creating CSV file: {e}')

    def pos_listener_callback(self, msg, index):
        """General callback for all robots' desired positions."""
        try:
            self.positions[index]['x'] = msg.x
            self.positions[index]['y'] = msg.y
            self.positions[index]['z'] = msg.z
        except Exception as e:
            self.get_logger().error(f'Error in position callback for Robot {index}: {e}')
    
    def odom_listener_callback(self, msg, index):
        """General callback for all robots' odometry data."""
        try:
            self.odometry_data[index]['position'] = msg.pose.pose.position
            self.odometry_data[index]['orientation'] = msg.pose.pose.orientation
            self.odometry_data[index]['linear_velocity'] = msg.twist.twist.linear
            self.odometry_data[index]['angular_velocity'] = msg.twist.twist.angular

            # Save the data to CSV at each time step
            self.save_data_to_csv()
            self.get_logger().info(f'Received and saved odometry for Robot {index}: Position: {msg.pose.pose.position}')
        except Exception as e:
            self.get_logger().error(f'Error in odometry callback for Robot {index}: {e}')

    def publish_messages(self):
        """Publish trajectory messages for each robot."""
        for i in range(self.N):
            if self.positions[i]['x'] is not None and self.positions[i]['y'] is not None and self.positions[i]['z'] is not None:
                self.publish_trajectory(self.positions[i], self.publishers_traj_params[i], f'Agent_{i}')

    def publish_trajectory(self, position, publisher, agent_name):
        """Helper function to publish a trajectory for a single robot."""
        try:
            msg = Trajectory()
            point = TrajectoryPoint()

            # Set position
            x = float(position['x'])
            y = float(position['y'])
            z = float(self.height)

            point.positions = [x, y, z]
            point.velocities = [0.0, 0.0, 0.0]
            point.accelerations = [0.0, 0.0, 0.0]
            point.effort = [0.0, 0.0, 0.0]
            point.time_from_start.sec = int(0)
            point.time_from_start.nanosec = int(0)

            # Publish message
            msg.joint_names.append(agent_name)
            msg.points.append(point)
            publisher.publish(msg)

            self.get_logger().info(f'Publishing trajectory for {agent_name}: Positions: {point.positions}')
        except Exception as e:
            self.get_logger().error(f'Error in publish_trajectory for {agent_name}: {e}')

    def save_data_to_csv(self):
        """Save the odometry data for all robots to the CSV file."""
        try:
            with open(self.csv_filename, mode='a', newline='') as csvfile:
                fieldnames = ['time_stamp']
                for i in range(self.N):
                    fieldnames.extend([
                        f'robot_{i}_position_x', f'robot_{i}_position_y', f'robot_{i}_position_z', 
                        f'robot_{i}_orientation_x', f'robot_{i}_orientation_y', f'robot_{i}_orientation_z', f'robot_{i}_orientation_w',
                        f'robot_{i}_linear_velocity_x', f'robot_{i}_linear_velocity_y', f'robot_{i}_linear_velocity_z',
                        f'robot_{i}_angular_velocity_x', f'robot_{i}_angular_velocity_y', f'robot_{i}_angular_velocity_z'
                    ])
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                row = {'time_stamp': self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9}

                # Add odometry data for each robot to the row
                for i, data in self.odometry_data.items():
                    row[f'robot_{i}_position_x'] = data['position'].x if data['position'] else None
                    row[f'robot_{i}_position_y'] = data['position'].y if data['position'] else None
                    row[f'robot_{i}_position_z'] = data['position'].z if data['position'] else None
                    row[f'robot_{i}_orientation_x'] = data['orientation'].x if data['orientation'] else None
                    row[f'robot_{i}_orientation_y'] = data['orientation'].y if data['orientation'] else None
                    row[f'robot_{i}_orientation_z'] = data['orientation'].z if data['orientation'] else None
                    row[f'robot_{i}_orientation_w'] = data['orientation'].w if data['orientation'] else None
                    row[f'robot_{i}_linear_velocity_x'] = data['linear_velocity'].x if data['linear_velocity'] else None
                    row[f'robot_{i}_linear_velocity_y'] = data['linear_velocity'].y if data['linear_velocity'] else None
                    row[f'robot_{i}_linear_velocity_z'] = data['linear_velocity'].z if data['linear_velocity'] else None
                    row[f'robot_{i}_angular_velocity_x'] = data['angular_velocity'].x if data['angular_velocity'] else None
                    row[f'robot_{i}_angular_velocity_y'] = data['angular_velocity'].y if data['angular_velocity'] else None
                    row[f'robot_{i}_angular_velocity_z'] = data['angular_velocity'].z if data['angular_velocity'] else None

                writer.writerow(row)
        except Exception as e:
            self.get_logger().error(f'Error saving odometry data to CSV: {e}')

def main(args=None):
    rclpy.init(args=args)
    traj_publisher = TrajectoryPublisher()
    rclpy.spin(traj_publisher)
    traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

