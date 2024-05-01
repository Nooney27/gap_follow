import rclpy
from rclpy.node import Node
import math

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class ReactiveFollowGap(Node):
    """ 
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('reactive_node')
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
       

        # TODO: Subscribe to LIDAR
        self.lidar_subscriptions = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        # TODO: Publish to drive
        self.publisher_ = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        print("ReactiveFollowGap Node initialized successfully.")


    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        ranges = np.array(ranges)
        window = 5
        #print(ranges)
        #print(np.ones(window)/window)
        smoothed_ranges = np.convolve(ranges, np.ones(window)/window, mode='valid')
        #print(smoothed_ranges)
        proc_ranges = np.where(smoothed_ranges > 20, 0, smoothed_ranges)
        #print(proc_ranges)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
        max_gap_start = 0
        max_gap_end = 0
        max_gap = 0
        curr_gap_start = None
        curr_gap_end = None

        for i, value in enumerate(free_space_ranges):
            if value > 0:
                if curr_gap_start is None:
                    curr_gap_start = i
                curr_gap_end = i
            else:
                if curr_gap_start is not None:
                    curr_gap = curr_gap_end - curr_gap_start + 1
                    if curr_gap > max_gap:
                        max_gap = curr_gap
                        max_gap_start = curr_gap_start
                        max_gap_end = curr_gap_end
                    curr_gap_start = None
                    # curr_gap_end = None

        if curr_gap_start is not None:
            curr_gap = curr_gap_end - curr_gap_start + 1
            if curr_gap > max_gap:
                max_gap = curr_gap
                max_gap_start = curr_gap_start
                max_gap_end = curr_gap_end

        return max_gap_start, max_gap_end
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	    Naive: Choose the furthest point within ranges and go there
        """
        best_point = start_i
        max_distance = 0
        for i in range(start_i, end_i+1):
            if ranges[i] > max_distance:
                max_distance = ranges[i]
                best_point = i
        return best_point
        
    def calculate_steering_angle(self, best_point_index, total_points):

        field_of_view = 95.0  # degrees

        # Calculate the angle per point
        angle_per_point = field_of_view / total_points

        # Calculate the angle offset from the center
        angle_from_center = (best_point_index - total_points / 2) * angle_per_point

        # Convert the angle from degrees to radians
        steering_angle = math.radians(angle_from_center)

        return steering_angle



    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges  # Assuming ranges is a list or numpy array of float distances from LiDAR
        bubble_radius = 3 # 3  # Define bubble_radius before using it

        #valid_ranges = [r for r in ranges if r >= bubble_radius]

        # Ensure that we only consider non-zero ranges and ranges above the bubble_radius
        closest_point = min((r for r in ranges if r >= bubble_radius), default=float('inf'))

        # Check if there are valid points to process
        if closest_point == float('inf'):
            self.get_logger().warn('No valid points outside the bubble radius')
            return  # Optionally skip processing if no points are valid

        proc_ranges = self.preprocess_lidar(ranges)

        # Eliminate all points inside 'bubble' (set them to zero)
        proc_ranges = np.array([0 if r < bubble_radius else r for r in proc_ranges])

        # Find max length gap
        start_i, end_i = self.find_max_gap(proc_ranges)

        # Find the best point in the gap
        best_point_index = self.find_best_point(start_i, end_i, proc_ranges)

        mid_index = len(proc_ranges) // 2
        front_distance = proc_ranges[mid_index] if mid_index < len(proc_ranges) else 0

        left_index = int(mid_index - len(proc_ranges) * 0.25)
        right_index = int(mid_index + len(proc_ranges) * 0.25)
        left_distance = proc_ranges[left_index] if left_index > 0 else 0
        right_distance = proc_ranges[right_index] if right_index < len(proc_ranges) else 0

        curvature = abs(left_distance - right_distance) / (left_distance + right_distance + 0.001)  # avoid division by zero
        print("curvature: ", curvature)
        print("front_distance: ", front_distance)
        
        if front_distance > 6.0 and front_distance < 20.0:
            speed = 6.0
        elif front_distance <= 6.0 and front_distance > 2.3:
            speed = 2.7
        else: 
            speed = 1.0
        print("speed:", speed)
        '''
        if front_distance <= 2.0:
            speed = 3.0
        else:
            speed = 1.2
        '''
        # Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"  # or use the appropriate frame ID
        drive_msg.drive.speed = speed # 1.5  # Set speed, consider dynamic speed adjustment
        drive_msg.drive.steering_angle = self.calculate_steering_angle(best_point_index, len(ranges))

        self.publisher_.publish(drive_msg)
    

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()