import rclpy
from rclpy.node import Node
from vicon_receiver.msg import Position
from geometry_msgs.msg import Point, TransformStamped
from trajectory_msgs.msg import JointTrajectory as Trajectory, JointTrajectoryPoint as TrajectoryPoint
import csv
import os


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('states_processing')

        self.N = 4  # Number of robots

        # Initialize position attributes for each robot
        self.positions = {i: {'x': None, 'y': None, 'z': None} for i in range(self.N)}
        self.vicon_data = {i: {'position': None, 'orientation': None, 'time_stamp': None} for i in range(self.N)}

        # Create subscribers and publishers for each robot
        self.pos_subscribers = {}
        self.publishers_traj_params = {}
        self.vicon_subscribers = {}

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

            # Vicon data subscribers
            topic_name = f'/vicon/cf{i+1}/cf{i+1}'
            self.vicon_subscribers[i] = self.create_subscription(
                Position, topic_name, 
                lambda msg, idx=i: self.vicon_listener_callback(msg, idx), 
                10)

        # Set setpoint height for all robots
        self.height = 1  # meter

        # Create a timer to publish messages periodically
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

        # Create a directory for storing CSV files if it doesn't exist
        self.csv_directory = 'vicon_data'
        os.makedirs(self.csv_directory, exist_ok=True)

        # Create a CSV file and write the header
        self.csv_filename = os.path.join(self.csv_directory, 'vicon_data.csv')
        self.create_csv_file()

    def create_csv_file(self):
        """Create a new CSV file and write the header."""
        try:
            with open(self.csv_filename, mode='w', newline='') as csvfile:
                fieldnames = ['time_stamp']
                # Add field names for each vehicle's Vicon data
                for i in range(self.N):
                    fieldnames.extend([
                        f'vehicle_{i}_position_x', f'vehicle_{i}_position_y', f'vehicle_{i}_position_z',
                        f'vehicle_{i}_orientation_x', f'vehicle_{i}_orientation_y', f'vehicle_{i}_orientation_z', f'vehicle_{i}_orientation_w'
                    ])
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
            self.get_logger().info(f"CSV file created at {self.csv_filename}")
        except Exception as e:
            self.get_logger().error(f'Error creating CSV file: {e}')

    def vicon_listener_callback(self, msg, vehicle_index):
        """Callback for the Vicon data for each vehicle."""
        try:
            # Log when the callback is triggered
            self.get_logger().info(f'Vicon callback triggered for vehicle {vehicle_index}')

            # Store the timestamp
            self.vicon_data[vehicle_index]['time_stamp'] = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9

            # Store the position and orientation from the Position message
            self.vicon_data[vehicle_index]['position'] = {
                'x': msg.x_trans,
                'y': msg.y_trans,
                'z': msg.z_trans
            }
            self.vicon_data[vehicle_index]['orientation'] = {
                'x': msg.x_rot,
                'y': msg.y_rot,
                'z': msg.z_rot,
                'w': msg.w
            }

            # Save the data to CSV
            self.save_data_to_csv()
            self.get_logger().info(f'Successfully saved Vicon data for Vehicle {vehicle_index}')
        except Exception as e:
            self.get_logger().error(f'Error in Vicon listener callback for Vehicle')

    def pos_listener_callback(self, msg, index):
        """General callback for all robots' desired positions."""
        try:
            self.positions[index]['x'] = msg.x
            self.positions[index]['y'] = msg.y
            self.positions[index]['z'] = msg.z
        except Exception as e:
            self.get_logger().error(f'Error in position callback for Robot {index}: {e}')

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
        """Save the Vicon data for all vehicles to the CSV file."""
        try:
            with open(self.csv_filename, mode='a', newline='') as csvfile:
                fieldnames = ['time_stamp']
                for i in range(self.N):
                    fieldnames.extend([
                        f'vehicle_{i}_position_x', f'vehicle_{i}_position_y', f'vehicle_{i}_position_z',
                        f'vehicle_{i}_orientation_x', f'vehicle_{i}_orientation_y', f'vehicle_{i}_orientation_z', f'vehicle_{i}_orientation_w'
                    ])
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

                # Prepare the data row
                row = {'time_stamp': self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec * 1e-9}

                # Add Vicon data for each vehicle to the row
                for i, data in self.vicon_data.items():
                    row[f'vehicle_{i}_position_x'] = data['position']['x'] if data['position'] else None
                    row[f'vehicle_{i}_position_y'] = data['position']['y'] if data['position'] else None
                    row[f'vehicle_{i}_position_z'] = data['position']['z'] if data['position'] else None
                    row[f'vehicle_{i}_orientation_x'] = data['orientation']['x'] if data['orientation'] else None
                    row[f'vehicle_{i}_orientation_y'] = data['orientation']['y'] if data['orientation'] else None
                    row[f'vehicle_{i}_orientation_z'] = data['orientation']['z'] if data['orientation'] else None
                    row[f'vehicle_{i}_orientation_w'] = data['orientation']['w'] if data['orientation'] else None

                writer.writerow(row)
                self.get_logger().info(f'Saved Vicon data to CSV for timestamp {row["time_stamp"]}')
        except Exception as e:
            self.get_logger().error(f'Error saving Vicon data to CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    traj_publisher = TrajectoryPublisher()
    rclpy.spin(traj_publisher)
    traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
