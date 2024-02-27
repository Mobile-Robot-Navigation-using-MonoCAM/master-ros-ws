import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2

def callback(point_cloud_msg):
    # Create an occupancy grid with the same dimensions as the point cloud
    occupancy_grid = OccupancyGrid()
    occupancy_grid.header = point_cloud_msg.header
    occupancy_grid.info.width = point_cloud_msg.width
    occupancy_grid.info.height = point_cloud_msg.height
    occupancy_grid.info.resolution = 10  # Set the resolution of the occupancy grid as desired

    # Initialize the occupancy grid data with zeros
    occupancy_grid.data = [0] * (occupancy_grid.info.width * occupancy_grid.info.height)

    # Iterate over the points in the point cloud
    for point in pc2.read_points(point_cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        # Get the x and y coordinates of the point
        print (point)
        x = int(point[0] / occupancy_grid.info.resolution)
        y = int(point[1] / occupancy_grid.info.resolution)

        # Get the index of the cell in the occupancy grid
        cell_index = (y * occupancy_grid.info.width) + x
        #print("x:{} and y:{}".format(x,y))
        # Set the occupancy of the cell
        occupancy_grid.data[cell_index] = 100
    #print(occupancy_grid)
    # Publish the occupancy grid
    pub.publish(occupancy_grid)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node("occupancy_grid_generator")

    # Create a publisher for the occupancy grid
    pub = rospy.Publisher("/gridmap", OccupancyGrid, queue_size=10)

    # Subscribe to the point cloud topic
    rospy.Subscriber("/orb_slam2_mono/map_points", PointCloud2, callback)

    # Spin until the node is shutdown
    rospy.spin()
