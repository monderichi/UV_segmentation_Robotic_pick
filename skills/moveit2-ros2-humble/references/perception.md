# Perception Pipelines Reference

## Table of Contents
1. [Point Cloud Processing](#point-cloud-processing)
2. [Object Detection Integration](#object-detection-integration)
3. [Depth Image Handling](#depth-image-handling)
4. [TF and Camera Calibration](#tf-and-camera-calibration)

## Point Cloud Processing

### Realsense Launch Configuration

```python
# In your launch file
realsense_node = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    parameters=[{
        'enable_color': True,
        'enable_depth': True,
        'enable_pointcloud': True,
        'pointcloud_texture_stream': 'RS2_STREAM_COLOR',
        'align_depth.enable': True,
        'depth_module.profile': '640x480x30',
        'rgb_camera.profile': '640x480x30',
        'clip_distance': 2.0,
    }],
    remappings=[
        ('/camera/camera/depth/color/points', '/camera/depth/color/points'),
    ]
)
```

### Voxel Grid Filtering

```python
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

def voxel_filter(points, voxel_size=0.01):
    """Downsample point cloud using voxel grid filter"""
    voxel_grid = {}
    for point in points:
        voxel = tuple(np.floor(point / voxel_size).astype(int))
        if voxel not in voxel_grid:
            voxel_grid[voxel] = []
        voxel_grid[voxel].append(point)
    
    # Return centroids
    filtered = []
    for voxel_points in voxel_grid.values():
        filtered.append(np.mean(voxel_points, axis=0))
    return np.array(filtered)

# Usage in callback
def cloud_callback(self, msg: PointCloud2):
    points = np.array(list(pc2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )))
    filtered = voxel_filter(points, voxel_size=0.005)
```

### Euclidean Clustering

```python
from sklearn.cluster import DBSCAN

def cluster_objects(points, eps=0.05, min_samples=10):
    """Cluster points into objects using DBSCAN"""
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = clustering.labels_
    
    objects = []
    for label in set(labels):
        if label == -1:  # Noise
            continue
        cluster_points = points[labels == label]
        centroid = np.mean(cluster_points, axis=0)
        objects.append({
            'centroid': centroid,
            'points': cluster_points,
            'label': label
        })
    return objects
```

### Plane Segmentation (RANSAC)

```python
def remove_table_plane(points, threshold=0.01):
    """Remove table plane using simple RANSAC"""
    # Find dominant plane (table)
    best_inliers = []
    best_eq = None
    
    for _ in range(100):  # RANSAC iterations
        # Sample 3 random points
        idx = np.random.choice(len(points), 3, replace=False)
        p1, p2, p3 = points[idx]
        
        # Compute plane equation
        normal = np.cross(p2 - p1, p3 - p1)
        normal = normal / np.linalg.norm(normal)
        d = -np.dot(normal, p1)
        
        # Find inliers
        distances = np.abs(np.dot(points, normal) + d)
        inliers = np.where(distances < threshold)[0]
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_eq = (normal, d)
    
    # Return points above table
    above_threshold = 0.02  # 2cm above table
    mask = np.dot(points, best_eq[0]) + best_eq[1] < -above_threshold
    return points[mask]
```

## Object Detection Integration

### YOLO + Depth Fusion

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
import numpy as np

class ObjectDetector3D(Node):
    def __init__(self):
        super().__init__('object_detector_3d')
        
        self.bridge = CvBridge()
        self.camera_info = None
        self.depth_image = None
        
        # Subscribers
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', 
                                 self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw',
                                 self.depth_callback, 10)
        self.create_subscription(Detection2DArray, '/yolo/detections',
                                 self.detection_callback, 10)
        
        self.pose_pub = self.create_publisher(PoseArray, '/object_poses', 10)
    
    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Extract camera matrix
        self.K = np.array(msg.k).reshape(3, 3)
    
    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
    
    def detection_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            return
        
        poses = PoseArray()
        poses.header = msg.header
        
        for detection in msg.detections:
            bbox = detection.bbox
            center_x = int(bbox.center.position.x)
            center_y = int(bbox.center.position.y)
            
            # Get depth at center
            depth = self.depth_image[center_y, center_x] / 1000.0  # mm to m
            
            if depth > 0:
                # Back-project to 3D
                x = (center_x - self.K[0, 2]) * depth / self.K[0, 0]
                y = (center_y - self.K[1, 2]) * depth / self.K[1, 1]
                z = depth
                
                pose = Pose()
                pose.position.x = x
                pose.position.y = y
                pose.position.z = z
                pose.orientation.w = 1.0
                
                poses.poses.append(pose)
        
        self.pose_pub.publish(poses)

def main():
    rclpy.init()
    node = ObjectDetector3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### ArUco Marker Detection

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',
                                 self.camera_info_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                                 self.image_callback, 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)
        
        # Marker size in meters
        self.marker_size = 0.05
    
    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
    
    def image_callback(self, msg):
        if self.camera_matrix is None:
            return
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            
            for i, marker_id in enumerate(ids):
                # Publish TF
                t = TransformStamped()
                t.header = msg.header
                t.child_frame_id = f'aruco_marker_{marker_id[0]}'
                t.transform.translation.x = float(tvecs[i][0][0])
                t.transform.translation.y = float(tvecs[i][0][1])
                t.transform.translation.z = float(tvecs[i][0][2])
                
                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i])
                q = self.rotation_matrix_to_quaternion(rot_matrix)
                t.transform.rotation.x = q[0]
                t.transform.rotation.y = q[1]
                t.transform.rotation.z = q[2]
                t.transform.rotation.w = q[3]
                
                self.tf_broadcaster.sendTransform(t)
                
                # Publish pose
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = t.transform.translation.x
                pose.pose.position.y = t.transform.translation.y
                pose.pose.position.z = t.transform.translation.z
                pose.pose.orientation = t.transform.rotation
                self.pose_pub.publish(pose)
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion"""
        trace = np.trace(R)
        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        return [x, y, z, w]

def main():
    rclpy.init()
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Depth Image Handling

### Depth to Point Cloud Conversion

```python
def depth_to_pointcloud(depth_image, camera_info):
    """Convert depth image to point cloud"""
    fx = camera_info.k[0]
    fy = camera_info.k[4]
    cx = camera_info.k[2]
    cy = camera_info.k[5]
    
    height, width = depth_image.shape
    
    # Create coordinate grids
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    # Back-project
    z = depth_image / 1000.0  # mm to m
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    # Stack and filter invalid points
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    valid = (z.flatten() > 0) & (z.flatten() < 10)  # 0-10m range
    
    return points[valid]
```

## TF and Camera Calibration

### Static Camera Transform Publisher

```python
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

def publish_camera_tf(self):
    """Publish static transform from robot base to camera"""
    broadcaster = StaticTransformBroadcaster(self)
    
    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'camera_link'
    
    # Camera position relative to base
    t.transform.translation.x = 0.5
    t.transform.translation.y = 0.0
    t.transform.translation.z = 1.0
    
    # Camera orientation (looking at robot)
    q = tf_transformations.quaternion_from_euler(0, 0.785, 3.14159)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    broadcaster.sendTransform(t)
```

### Hand-Eye Calibration (Eye-in-Hand)

```python
#!/usr/bin/env python3
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class HandEyeCalibration:
    """Eye-in-hand calibration for camera mounted on robot wrist"""
    
    def __init__(self):
        self.R_gripper2base = []
        self.t_gripper2base = []
        self.R_target2cam = []
        self.t_target2cam = []
    
    def add_pose(self, gripper_pose, target_pose):
        """
        Add a pose pair for calibration
        gripper_pose: (R, t) of gripper relative to base
        target_pose: (R, t) of calibration target relative to camera
        """
        self.R_gripper2base.append(gripper_pose[0])
        self.t_gripper2base.append(gripper_pose[1])
        self.R_target2cam.append(target_pose[0])
        self.t_target2cam.append(target_pose[1])
    
    def calibrate(self):
        """Perform hand-eye calibration"""
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base,
            self.t_gripper2base,
            self.R_target2cam,
            self.t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        return R_cam2gripper, t_cam2gripper
```
