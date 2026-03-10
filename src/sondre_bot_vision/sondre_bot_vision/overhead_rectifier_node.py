#!/usr/bin/env python3

import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge


class OverheadRectifierNode(Node):
    def __init__(self):
        super().__init__('overhead_rectifier_node')

        self.declare_parameter('board_width_mm', 3000)
        self.declare_parameter('board_height_mm', 2000)
        self.declare_parameter('tag_size_mm', 100)

        self.declare_parameter('image_topic', '/overhead_cam/image')
        self.declare_parameter('camera_info_topic', '/overhead_cam/camera_info')
        self.declare_parameter('warped_topic', '/vision/board_warped')
        self.declare_parameter('debug_topic', '/vision/board_debug')
        self.declare_parameter('robot_pose_topic', '/vision/robot_pose')

        self.declare_parameter('cache_h_seconds', 0.75)

        self.declare_parameter('fixed_ids', [20, 21, 22, 23])
        self.declare_parameter('fixed_x_mm', [600.0, 2400.0, 600.0, 2400.0])
        self.declare_parameter('fixed_y_mm', [1400.0, 1400.0, 600.0, 600.0])
        self.declare_parameter('fixed_yaw_deg', [0.0, 0.0, 0.0, 0.0])

        self.declare_parameter('robot_tag_id', 71)
        self.declare_parameter('robot_tag_size_m', 0.10)
        self.declare_parameter('robot_tag_yaw_offset_deg', 0.0)

        self.board_width_mm = int(self.get_parameter('board_width_mm').value)
        self.board_height_mm = int(self.get_parameter('board_height_mm').value)
        self.tag_size_mm = float(self.get_parameter('tag_size_mm').value)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.warped_topic = str(self.get_parameter('warped_topic').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.robot_pose_topic = str(self.get_parameter('robot_pose_topic').value)

        self.cache_h_seconds = float(self.get_parameter('cache_h_seconds').value)

        self.robot_tag_id = int(self.get_parameter('robot_tag_id').value)
        self.robot_tag_size_m = float(self.get_parameter('robot_tag_size_m').value)
        self.robot_tag_yaw_offset_rad = math.radians(
            float(self.get_parameter('robot_tag_yaw_offset_deg').value)
        )

        fixed_ids = list(self.get_parameter('fixed_ids').value)
        fixed_x = list(self.get_parameter('fixed_x_mm').value)
        fixed_y = list(self.get_parameter('fixed_y_mm').value)
        fixed_yaw = list(self.get_parameter('fixed_yaw_deg').value)

        if not (len(fixed_ids) == len(fixed_x) == len(fixed_y) == len(fixed_yaw)):
            raise ValueError('fixed_ids, fixed_x_mm, fixed_y_mm, fixed_yaw_deg must have same length')

        self.fixed_tags = {}
        for i, tag_id in enumerate(fixed_ids):
            self.fixed_tags[int(tag_id)] = {
                'x_mm': float(fixed_x[i]),
                'y_mm': float(fixed_y[i]),
                'yaw_deg': float(fixed_yaw[i]),
            }

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )

        self.warped_pub = self.create_publisher(Image, self.warped_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.robot_pose_pub = self.create_publisher(Pose2D, self.robot_pose_topic, 10)

        self.last_h = None
        self.last_h_time = None

        self.camera_matrix = None
        self.dist_coeffs = None

        self.last_robot_pose = None
        self.last_cam_rvec = None
        self.last_cam_tvec = None
        self.last_cam_pose_time = None

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_100)
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.get_logger().info(f'Listening on {self.image_topic}')
        self.get_logger().info(f'Listening on {self.camera_info_topic}')
        self.get_logger().info(f'Publishing warped image on {self.warped_topic}')
        self.get_logger().info(f'Publishing debug image on {self.debug_topic}')
        self.get_logger().info(f'Publishing robot pose on {self.robot_pose_topic}')
        self.get_logger().info(f'Fixed tags: {sorted(self.fixed_tags.keys())}')
        self.get_logger().info(f'Robot tag ID: {self.robot_tag_id}')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    @staticmethod
    def wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def homography_still_valid(self) -> bool:
        if self.last_h is None or self.last_h_time is None:
            return False
        age = (self.get_clock().now() - self.last_h_time).nanoseconds / 1e9
        return age <= self.cache_h_seconds

    def camera_pose_still_valid(self) -> bool:
        if self.last_cam_rvec is None or self.last_cam_tvec is None or self.last_cam_pose_time is None:
            return False
        age = (self.get_clock().now() - self.last_cam_pose_time).nanoseconds / 1e9
        return age <= self.cache_h_seconds

    def dst_tag_corners_px(self, cx_mm: float, cy_mm: float) -> np.ndarray:
        px = self.board_width_mm - cx_mm
        py = cy_mm
        h = self.tag_size_mm / 2.0

        return np.array([
            [px + h, py - h],
            [px - h, py - h],
            [px - h, py + h],
            [px + h, py + h],
        ], dtype=np.float32)

    def fixed_tag_corners_world_3d(self, cx_mm: float, cy_mm: float) -> np.ndarray:
        """
        Gazebo/world frame:
          x right
          y up
          z up out of table
        Markers lie on z=0 plane.

        Corner order:
          top-left, top-right, bottom-right, bottom-left
        with marker "top" pointing +y in world.
        """
        cx = cx_mm / 1000.0
        cy = cy_mm / 1000.0
        h = self.tag_size_mm / 2000.0

        return np.array([
            [cx - h, cy + h, 0.0],
            [cx + h, cy + h, 0.0],
            [cx + h, cy - h, 0.0],
            [cx - h, cy - h, 0.0],
        ], dtype=np.float32)

    def robot_tag_corners_local_3d(self) -> np.ndarray:
        """
        Square marker corners centered at tag center.
        Order required by SOLVEPNP_IPPE_SQUARE:
          top-left, top-right, bottom-right, bottom-left
        """
        h = self.robot_tag_size_m / 2.0
        return np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float32)

    def solve_camera_pose_from_fixed_tags(self, image_pts: np.ndarray, world_pts: np.ndarray):
        if self.camera_matrix is None:
            return None, None

        dist = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5, 1), dtype=np.float64)

        ok, rvec, tvec = cv2.solvePnP(
            world_pts.astype(np.float32),
            image_pts.astype(np.float32),
            self.camera_matrix,
            dist,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        if not ok:
            return None, None

        return rvec, tvec

    def choose_best_robot_pose(self, rvecs, tvecs, reprojection_errors, R_world_optical, cam_pos_world):
        best = None
        best_score = None

        prev_xy = None
        prev_theta = None
        if self.last_robot_pose is not None:
            prev_xy = np.array([self.last_robot_pose.x, self.last_robot_pose.y], dtype=np.float64)
            prev_theta = self.last_robot_pose.theta

        for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            R_optical_tag, _ = cv2.Rodrigues(rvec)
            t_optical_tag = tvec.reshape(3, 1)

            tag_center_world = cam_pos_world + R_world_optical @ t_optical_tag
            x = float(tag_center_world[0, 0])
            y = float(tag_center_world[1, 0])

            R_world_tag = R_world_optical @ R_optical_tag
            forward_world = R_world_tag @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
            theta = self.wrap_angle(
                math.atan2(forward_world[1], forward_world[0]) + self.robot_tag_yaw_offset_rad
            )

            reproj = float(reprojection_errors[i]) if reprojection_errors is not None else 0.0
            score = reproj

            if prev_xy is not None:
                score += 5.0 * np.linalg.norm(np.array([x, y]) - prev_xy)
            if prev_theta is not None:
                score += 1.0 * abs(self.wrap_angle(theta - prev_theta))

            # reject impossible arena positions
            if not (0.0 <= x <= 3.0 and 0.0 <= y <= 2.0):
                score += 1000.0

            # reject obviously wrong height
            z = float(tag_center_world[2, 0])
            if not (0.20 <= z <= 0.80):
                score += 1000.0

            if best_score is None or score < best_score:
                best_score = score
                best = (x, y, theta)

        return best

    def publish_robot_pose_from_raw(self, cam_rvec, cam_tvec, robot_corners):
        if self.camera_matrix is None:
            return
        if cam_rvec is None or cam_tvec is None:
            return
        if robot_corners is None:
            return

        dist = self.dist_coeffs if self.dist_coeffs is not None else np.zeros((5, 1), dtype=np.float64)

        robot_obj_pts = self.robot_tag_corners_local_3d()
        robot_img_pts = robot_corners.reshape(4, 2).astype(np.float32)

        ok, rvecs, tvecs, reprojection_errors = cv2.solvePnPGeneric(
            robot_obj_pts,
            robot_img_pts,
            self.camera_matrix,
            dist,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if not ok or len(rvecs) == 0:
            return

        # cam pose from solvePnP(object=world, image=raw):
        # X_cam = R_cw * X_world + t_cw
        R_cw, _ = cv2.Rodrigues(cam_rvec)
        t_cw = cam_tvec.reshape(3, 1)

        # invert -> world from optical
        R_world_optical = R_cw.T
        cam_pos_world = -R_world_optical @ t_cw

        best = self.choose_best_robot_pose(
            rvecs, tvecs, reprojection_errors,
            R_world_optical, cam_pos_world
        )

        if best is None:
            return

        x, y, theta = best

        pose = Pose2D()
        pose.x = x
        pose.y = y
        pose.theta = theta

        self.last_robot_pose = pose
        self.robot_pose_pub.publish(pose)

    def image_callback(self, msg: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge failed: {e}')
            return

        debug = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params
        )

        H_use = None
        cam_rvec_use = None
        cam_tvec_use = None
        status_text = 'NO TAGS'
        found_fixed_ids = []
        robot_corners = None

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(debug, corners, ids.reshape(-1, 1))

            image_pts_2d = []
            board_pts_2d = []

            image_pts_world = []
            world_pts = []

            for marker_corners, marker_id in zip(corners, ids_flat):
                marker_id = int(marker_id)

                if marker_id == self.robot_tag_id:
                    robot_corners = marker_corners.copy()

                if marker_id not in self.fixed_tags:
                    continue

                found_fixed_ids.append(marker_id)

                img4 = marker_corners.reshape(4, 2).astype(np.float32)
                tag_cfg = self.fixed_tags[marker_id]

                dst4 = self.dst_tag_corners_px(tag_cfg['x_mm'], tag_cfg['y_mm'])
                obj4_world = self.fixed_tag_corners_world_3d(tag_cfg['x_mm'], tag_cfg['y_mm'])

                image_pts_2d.append(img4)
                board_pts_2d.append(dst4)

                image_pts_world.append(img4)
                world_pts.append(obj4_world)

            if len(image_pts_2d) >= 2:
                image_pts_2d = np.vstack(image_pts_2d)
                board_pts_2d = np.vstack(board_pts_2d)

                H, _ = cv2.findHomography(image_pts_2d, board_pts_2d, cv2.RANSAC, 5.0)

                if H is not None:
                    self.last_h = H
                    self.last_h_time = self.get_clock().now()
                    H_use = H
                    status_text = f'LIVE H  fixed={sorted(found_fixed_ids)}'
                else:
                    status_text = f'FAILED H  fixed={sorted(found_fixed_ids)}'
            else:
                status_text = f'NOT ENOUGH FIXED TAGS  fixed={sorted(found_fixed_ids)}'

            if len(image_pts_world) >= 2:
                image_pts_world = np.vstack(image_pts_world)
                world_pts = np.vstack(world_pts)

                cam_rvec, cam_tvec = self.solve_camera_pose_from_fixed_tags(image_pts_world, world_pts)
                if cam_rvec is not None:
                    self.last_cam_rvec = cam_rvec
                    self.last_cam_tvec = cam_tvec
                    self.last_cam_pose_time = self.get_clock().now()
                    cam_rvec_use = cam_rvec
                    cam_tvec_use = cam_tvec

        if H_use is None and self.homography_still_valid():
            H_use = self.last_h
            if found_fixed_ids:
                status_text = f'CACHED H  fixed={sorted(found_fixed_ids)}'
            else:
                status_text = 'CACHED H  no fixed tags'

        if cam_rvec_use is None and self.camera_pose_still_valid():
            cam_rvec_use = self.last_cam_rvec
            cam_tvec_use = self.last_cam_tvec

        if H_use is not None:
            warped = cv2.warpPerspective(
                frame,
                H_use,
                (self.board_width_mm, self.board_height_mm)
            )
            warped_msg = self.bridge.cv2_to_imgmsg(warped, encoding='bgr8')
            warped_msg.header = msg.header
            self.warped_pub.publish(warped_msg)

        if robot_corners is not None and cam_rvec_use is not None and cam_tvec_use is not None:
            self.publish_robot_pose_from_raw(cam_rvec_use, cam_tvec_use, robot_corners)

        if robot_corners is not None:
            rc = robot_corners.reshape(4, 2).astype(int)
            center = np.mean(rc, axis=0).astype(int)
            cv2.putText(
                debug,
                f'ROBOT {self.robot_tag_id}',
                (center[0] + 10, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA
            )

        cv2.putText(
            debug,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )

        debug_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OverheadRectifierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()