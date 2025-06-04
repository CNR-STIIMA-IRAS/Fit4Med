#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseArray, TransformStamped
from cv_bridge import CvBridge
import apriltag
import tf2_ros
from tf_transformations import quaternion_from_matrix

class AprilTagDetector(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # AprilTag detector setup
        self.detector = apriltag.Detector(apriltag.DetectorOptions(
            families='tag36h11',  # You can change this to tag25h9, tag16h5, etc.
            border=1,
            nthreads=4,
            quad_decimate=1.0,
            quad_blur=0.0,
            refine_edges=True,
            refine_decode=False,
            refine_pose=False,
            debug=False
        ))
        
        # Camera parameters (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # Declare and get parameters
        self.declare_parameter('tag_size', 0.162)  # 162mm tags
        self.tag_size = self.get_parameter('tag_size').get_parameter_value().double_value
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/rgb/image_raw', self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/rgb/camera_info', self.camera_info_callback, 10)
        
        # Publishers
        self.detection_pub = self.create_publisher(PoseArray, '/apriltag_detections', 10)
        self.image_pub = self.create_publisher(Image, '/apriltag_detection_image', 1)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info("AprilTag detector initialized")
    
    def camera_info_callback(self, msg):
        """Extract camera parameters from camera_info"""
        if not self.camera_info_received:
            self.camera_matrix = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ])
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info("Camera parameters received")
    
    def image_callback(self, msg):
        """Process incoming images for AprilTag detection"""
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for camera info...", throttle_duration_sec=1)
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags
            detections = self.detector.detect(gray)
            
            if detections:
                pose_array = PoseArray()
                pose_array.header = msg.header
                
                for detection in detections:
                    # Draw detection on image
                    self.draw_detection(cv_image, detection)
                    
                    # Estimate pose
                    pose = self.estimate_pose(detection)
                    if pose is not None:
                        pose_array.poses.append(pose.pose)
                        
                        # Publish TF transform
                        self.publish_tf(pose, detection.tag_id, msg.header)
                
                # Publish pose array
                if pose_array.poses:
                    self.detection_pub.publish(pose_array)
            
            # Publish annotated image
            self.publish_detection_image(cv_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def draw_detection(self, image, detection):
        """Draw AprilTag detection on image"""
        # Draw the tag outline
        pts = detection.corners.astype(int)
        cv2.polylines(image, [pts], True, (0, 255, 0), 2)
        
        # Draw tag center
        center = tuple(detection.center.astype(int))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
        # Draw tag ID
        cv2.putText(image, f"ID: {detection.tag_id}", 
                   (center[0] - 20, center[1] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
    
    def estimate_pose(self, detection):
        """Estimate 6DOF pose of detected AprilTag"""
        try:
            # 3D coordinates of tag corners in tag frame
            object_points = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [-self.tag_size/2,  self.tag_size/2, 0]
            ])
            
            # 2D image coordinates
            image_points = detection.corners
            
            # Solve PnP
            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, 
                self.camera_matrix, self.dist_coeffs
            )
            
            if success:
                # Convert to pose message
                pose_stamped = PoseStamped()
                pose_stamped.pose.position.x = tvec[0][0]
                pose_stamped.pose.position.y = tvec[1][0]
                pose_stamped.pose.position.z = tvec[2][0]
                
                # Convert rotation vector to quaternion
                rmat, _ = cv2.Rodrigues(rvec)
                # Create 4x4 transformation matrix
                T = np.eye(4)
                T[:3, :3] = rmat
                # Convert to quaternion
                quat = quaternion_from_matrix(T)
                pose_stamped.pose.orientation.x = quat[0]
                pose_stamped.pose.orientation.y = quat[1]
                pose_stamped.pose.orientation.z = quat[2]
                pose_stamped.pose.orientation.w = quat[3]
                
                return pose_stamped
                
        except Exception as e:
            self.get_logger().warn(f"Pose estimation failed: {str(e)}")
        
        return None
    
    def publish_tf(self, pose_stamped, tag_id, header):
        """Publish TF transform for detected tag"""
        try:
            transform = TransformStamped()
            transform.header = header
            transform.child_frame_id = f"apriltag_{tag_id}"
            
            transform.transform.translation.x = pose_stamped.pose.position.x
            transform.transform.translation.y = pose_stamped.pose.position.y
            transform.transform.translation.z = pose_stamped.pose.position.z
            transform.transform.rotation = pose_stamped.pose.orientation
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().warn(f"TF publishing failed: {str(e)}")
    
    def publish_detection_image(self, cv_image, header):
        """Publish annotated image"""
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            detection_msg.header = header
            self.image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().warn(f"Image publishing failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    detector = AprilTagDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()