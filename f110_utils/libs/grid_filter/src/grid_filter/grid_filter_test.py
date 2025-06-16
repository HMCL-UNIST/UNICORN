#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
from dynamic_reconfigure.msg import Config
from sensor_msgs import point_cloud2


class GridFilter:
    def __init__(self, map_topic=None, debug=False):
        self.resolution = None  # Map resolution (m/pixel)
        self.origin = None  # Map origin (x, y)
        self.map_data = None  # Store map data
        self.image = None  # Convert OccupancyGrid to OpenCV image
        self.eroded_image = None  # Processed image with erosion
        self.kernel_size = 3  # Default erosion kernel size
        self.debug = debug
        self.map_topic = map_topic  # User-specified map topic
        self.pub = None  # Publisher for the modified map
        self.point_pub = None  # Publisher for the filtered point cloud
        self.config_sub = rospy.Subscriber("/dyn_perception/parameter_updates", Config, self.dyn_param_cb)
        
        # Initialize publishers
        self.pub = rospy.Publisher("modified_map", OccupancyGrid, queue_size=10)
        self.point_pub = rospy.Publisher("filtered_points", PointCloud2, queue_size=10)

        # Ensure the topic is set before subscribing
        if self.map_topic:
            self.subscribe_to_map(self.map_topic)
        else:
            rospy.logwarn("No map topic provided. Waiting for user input.")

        # Subscribe to the point cloud data topic
        self.scan_sub = rospy.Subscriber("/scan_matched_points2", PointCloud2, self.point_cloud_callback)

    def dyn_param_cb(self, config):
        self.kernel_size = rospy.get_param("dyn_perception/filter_kernel_size")
        self.update_image()

    def subscribe_to_map(self, map_topic):
        """Subscribe to the given map topic."""
        rospy.loginfo(f"Subscribing to map topic: {map_topic}")
        rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        
    def map_callback(self, msg):
        """Process the received map data."""
        if self.image is None:
            rospy.logwarn("Received map data")

            self.resolution = msg.info.resolution
            self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

            width, height = msg.info.width, msg.info.height
            image = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Convert to 0-255 scale (255: obstacle, 0: free space)
            self.image = np.where(image == 100, 0, 255).astype(np.uint8)

            if self.debug:
                # Display the modified map
                cv2.imshow("Original Map with Points", self.image)
                cv2.waitKey(0)

            self.update_image()
            rospy.logwarn("Map image initialized.")

    def set_erosion_kernel_size(self, size):
        """Update kernel size and apply erosion."""
        self.kernel_size = size
        self.update_image()

    def update_image(self):
        """Apply erosion to the map."""
        if self.image is None:
            rospy.logwarn("Map image not initialized.")
            return

        # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (self.kernel_size, self.kernel_size))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.kernel_size, self.kernel_size))
        
        self.eroded_image = cv2.erode(self.image, kernel)

        if self.debug:
            cv2.imshow("Eroded Map", self.eroded_image)
            cv2.waitKey(0)

        self.publish_modified_map()

    def world_to_pixel(self, x, y):
        """Convert world coordinates to pixel coordinates."""
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        return px, py

    def is_point_inside(self, x, y):
        """Check if a world coordinate is inside an obstacle."""
        if self.eroded_image is None:
            return False

        px, py = self.world_to_pixel(x, y)

        if px < 0 or py < 0 or px >= self.eroded_image.shape[1] or py >= self.eroded_image.shape[0]:
            return False

        return self.eroded_image[py, px] == 255

    def publish_modified_map(self):
        """Publish the modified map as an OccupancyGrid."""
        if self.eroded_image is None:
            rospy.logwarn("No modified map available to publish.")
            return

        # Reverse the 0-255 scale back to 0 (free space), 100 (unknown), and 255 (obstacle) for OccupancyGrid
        modified_image = np.where(self.eroded_image == 255, 0, 100).astype(np.int8)

        # Create OccupancyGrid message
        modified_map = OccupancyGrid()
        modified_map.header = Header()
        modified_map.header.stamp = rospy.Time.now()
        modified_map.header.frame_id = "map"
        modified_map.info.resolution = self.resolution
        modified_map.info.width = self.eroded_image.shape[1]
        modified_map.info.height = self.eroded_image.shape[0]
        modified_map.info.origin.position.x = self.origin[0]
        modified_map.info.origin.position.y = self.origin[1]
        
        # Flatten the modified image and assign to the data field
        modified_map.data = modified_image.flatten().tolist()

        # Publish the modified map
        self.pub.publish(modified_map)

    def point_cloud_callback(self, msg):
        """Callback function to filter points based on the map."""
        # Convert PointCloud2 message to a list of points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        filtered_points = []

        # Filter points based on whether they are inside an obstacle (using map data)
        for point in points:
            x, y, z = point
            if self.is_point_inside(x, y):
                filtered_points.append([x, y, z])

        # Convert the filtered points back to PointCloud2 format
        if filtered_points:
            header = msg.header
            filtered_cloud = pc2.create_cloud_xyz32(header, filtered_points)
            # Publish the filtered point cloud
            self.point_pub.publish(filtered_cloud)


if __name__ == "__main__":
    rospy.init_node("map_filter_node")

    # User-defined topic and debug flag
    map_topic = "/map"  # Example topic name (adjust as needed)
    debug = False  # Set to True if you want to visualize the images

    # Create the GridFilter object
    map_filter = GridFilter(map_topic=map_topic, debug=debug)

    rospy.spin()
