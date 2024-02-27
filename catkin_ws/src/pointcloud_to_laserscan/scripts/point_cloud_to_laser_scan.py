import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from math import atan2, isnan, pi, cos, sin
from rospy import Time
from costmap_2d.msg import Costmap2D

# Global variables to store maximum distance coordinates
max_distance_x = 0.0
max_distance_y = 0.0

def convert_point_cloud_to_costmap(point_cloud_msg):
    # Convert the point cloud to a costmap-like representation
    costmap_msg = Costmap2D()
    costmap_msg.header = point_cloud_msg.header

    # Process each point in the point cloud
    for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        # Calculate the range (distance) and angle of the point
        x, y, _ = point
        range_value = (x ** 2 + y ** 2) ** 0.5
        angle_value = atan2(y, x)

        # Add the range and angle values to the costmap message
        costmap_msg.ranges.append(range_value)
        costmap_msg.intensities.append(angle_value)

    return costmap_msg

def generate_occupancy_grid():
    global max_distance_x, max_distance_y

    # Create an occupancy grid with the dimensions based on maximum distance coordinates
    occupancy_grid_msg = OccupancyGrid()
    occupancy_grid_msg.header.stamp = Time.now()
    occupancy_grid_msg.info.width = int(2 * max_distance_x)
    occupancy_grid_msg.info.height = int(2 * max_distance_y)
    occupancy_grid_msg.info.resolution = 1.0  # Set the resolution of the occupancy grid as desired

    # Initialize the occupancy grid data with unknown values (-1)
    occupancy_grid_msg.data = [-1] * (occupancy_grid_msg.info.width * occupancy_grid_msg.info.height)

    return occupancy_grid_msg

def update_occupancy_grid(point_cloud_msg, occupancy_grid_msg, angle, costmap_msg):
    for point, cost in zip(pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True), costmap_msg.ranges):
        x, y, _ = point

        if isnan(x) or isnan(y):
            continue

        # Apply rotation to the point
        rotated_x = x * cos(angle) - y * sin(angle)
        rotated_y = x * sin(angle) + y * cos(angle)

        # Convert rotated point coordinates to occupancy grid cell indices
        cell_x = int(rotated_x + max_distance_x)
        cell_y = int(rotated_y + max_distance_y)

        # Check if the cell indices are within the occupancy grid boundaries
        if 0 <= cell_x < occupancy_grid_msg.info.width and 0 <= cell_y < occupancy_grid_msg.info.height:
            # Calculate the cell index in the occupancy grid data
            cell_index = cell_y * occupancy_grid_msg.info.width + cell_x

            # Set the occupancy of the cell as occupied (100)
            occupancy_grid_msg.data[cell_index] = cost

    return occupancy_grid_msg

def callback(point_cloud_msg):
    global max_distance_x, max_distance_y

    # Check if maximum distance coordinates need to be updated
    if max_distance_x == 0.0 and max_distance_y == 0.0:
        # Initialize maximum distance coordinates
        for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, _ = point
            max_distance_x = max(max_distance_x, abs(x))
            max_distance_y = max(max_distance_y, abs(y))

    # Generate occupancy grid if not already initialized
    if max_distance_x != 0.0 and max_distance_y != 0.0:
        occupancy_grid_msg = generate_occupancy_grid()

        # Get the rotation angle (you need to update this based on your camera rotation mechanism)
        angle = 0.0  # Update this with the actual rotation angle

        # Convert point cloud to costmap
        costmap_msg = convert_point_cloud_to_costmap(point_cloud_msg)

        # Update occupancy grid based on the new point cloud, rotation, and costmap
        occupancy_grid_msg = update_occupancy_grid(point_cloud_msg, occupancy_grid_msg, angle, costmap_msg)

        # Publish the occupancy grid
        pub.publish(occupancy_grid_msg)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("occupancy_grid_generator")

    # Create a publisher for the occupancy grid
    pub = rospy.Publisher("/gridmap", OccupancyGrid, queue_size=10)

    # Subscribe to the point cloud topic
    rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, callback)

    # Spin until the node is shutdown
    rospy.spin()
