from pathlib import Path

import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as R

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image


class FeatPosePublisher(Node):
    def __init__(self):
        super().__init__("featpose_publisher")

        self.bridge = CvBridge()

        self.pose_publisher = self.create_publisher(PoseStamped, "/featpose/pose", 10)
        self.image_publisher = self.create_publisher(Image, "/featpose/image", 10)

    def load_model(self, model_type):
        package_share_directory = get_package_share_directory("featpose")
        model_path = Path(package_share_directory) / "model" / f"{model_type}.npz"
        model_data = np.load(model_path)

        des = model_data["descriptors"]
        des_to_vtx = model_data["descriptors_to_vertices"]
        vtx = model_data["vertices"]
        edg = model_data["edges"]

        return des, des_to_vtx, vtx, edg

    def match_features(self, frame_gray):
        kp, des = self.orb.detectAndCompute(frame_gray, None)

        if isinstance(self.matcher, cv.BFMatcher):
            matches = self.matcher.match(des, self.des)
            good_matches = sorted(matches, key=lambda m: m.distance)[:10]
        elif isinstance(self.matcher, cv.FlannBasedMatcher):
            matches = self.matcher.knnMatch(des, self.des, k=2)
            good_matches = [
                match[0]
                for match in matches
                if len(match) >= 2 and match[0].distance < 0.75 * match[1].distance
            ]

        return good_matches, kp

    def estimate_pose(self, good_matches, kp):
        train_idx = [m.trainIdx for m in good_matches]
        query_idx = [m.queryIdx for m in good_matches]

        objp = self.vtx[self.des_to_vtx[train_idx]]
        imgp = np.array([kp[i].pt for i in query_idx], dtype=np.float32)

        return cv.solvePnPRansac(objp, imgp, self.mtx, self.dist)[:3]

    def draw_pose(self, frame_bgr, rvec, tvec):
        imgp = cv.projectPoints(self.vtx, rvec, tvec, self.mtx, self.dist)[0]
        imgp = np.int32(imgp).reshape(-1, 2)

        if self.model_type == "cube":
            for i, j in self.edg[:4]:
                frame_bgr = cv.line(
                    frame_bgr, tuple(imgp[i]), tuple(imgp[j]), (0, 255, 0), 3
                )
            for i, j in self.edg[4:8]:
                frame_bgr = cv.line(
                    frame_bgr, tuple(imgp[i]), tuple(imgp[j]), (255, 0, 0), 3
                )
            for i, j in self.edg[8:]:
                frame_bgr = cv.line(
                    frame_bgr, tuple(imgp[i]), tuple(imgp[j]), (0, 0, 255), 3
                )

        return frame_bgr

    def publish_pose_message(self, rvec, tvec):
        pose_msg = PoseStamped()

        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "camera_link"

        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[1][0]
        pose_msg.pose.position.z = tvec[2][0]

        rmat = cv.Rodrigues(rvec)[0]
        quat = R.from_matrix(rmat).as_quat()

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.pose_publisher.publish(pose_msg)

    def publish_image_message(self, frame_bgr):
        image_msg = self.bridge.cv2_to_imgmsg(frame_bgr)

        self.image_publisher.publish(image_msg)
