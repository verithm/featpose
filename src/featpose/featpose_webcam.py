import numpy as np
import cv2 as cv

import rclpy

from featpose import FeatPosePublisher


class FeatPoseWebcamPublisher(FeatPosePublisher):
    def __init__(self):
        super().__init__()

        self.declare_parameter("model_type", "cube")
        self.declare_parameter("descriptor_matcher", "BF")
        self.declare_parameter(
            "camera_matrix",
            [
                50 * 1920 / 36,
                0.0,
                1920 / 2,
                0.0,
                50 * 1920 / 36,
                1080 / 2,
                0.0,
                0.0,
                1.0,
            ],
        )
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        self.model_type = (
            self.get_parameter("model_type").get_parameter_value().string_value
        )
        descriptor_matcher = (
            self.get_parameter("descriptor_matcher").get_parameter_value().string_value
        )
        camera_matrix = (
            self.get_parameter("camera_matrix").get_parameter_value().double_array_value
        )
        dist_coeffs = (
            self.get_parameter("dist_coeffs").get_parameter_value().double_array_value
        )

        self.des, self.des_to_vtx, self.vtx, self.edg = self.load_model(self.model_type)

        self.orb = cv.ORB_create()

        if descriptor_matcher == "BF":
            self.matcher = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        elif descriptor_matcher == "FLANN":
            index_params = dict(
                algorithm=6, table_number=6, key_size=12, multi_probe_level=1
            )
            search_params = dict(checks=50)
            self.matcher = cv.FlannBasedMatcher(index_params, search_params)
        else:
            raise ValueError(f"Invalid Descriptor Matcher: {descriptor_matcher}")

        self.mtx = np.array(camera_matrix, dtype=np.float32).reshape(3, 3)
        self.dist = np.array(dist_coeffs, dtype=np.float32)

        self.open_webcam_timer = self.create_timer(5.0, self.open_webcam_callback)

    def open_webcam_callback(self):
        self.cap = cv.VideoCapture(0)

        if self.cap.isOpened():
            self.open_webcam_timer.cancel()
            self.read_webcam_timer = self.create_timer(
                1.0 / 30, self.read_webcam_callback
            )
        else:
            self.cap.release()
            self.get_logger().error("Webcam Open Failed")

    def read_webcam_callback(self):
        ret_frame, frame_bgr = self.cap.read()

        if not ret_frame:
            self.get_logger().error("Frame Read Failed")
            return

        frame_gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        good_matches, kp = self.match_features(frame_gray)

        if len(good_matches) >= 4:
            ret_pose, rvec, tvec = self.estimate_pose(good_matches, kp)

            if ret_pose:
                self.draw_pose(frame_bgr, rvec, tvec)
                self.publish_pose_message(rvec, tvec)
            else:
                self.get_logger().warn("Pose Estimation Failed")

        else:
            self.get_logger().warn("Feature Match Failed")

        self.publish_image_message(frame_bgr)

    def close_webcam(self):
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()


def main():
    rclpy.init()

    featpose_webcam_publisher = FeatPoseWebcamPublisher()

    try:
        rclpy.spin(featpose_webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        featpose_webcam_publisher.close_webcam()
        featpose_webcam_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
