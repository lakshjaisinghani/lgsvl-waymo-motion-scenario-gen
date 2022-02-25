from lib2to3.pytree import Base
import lgsvl
from lgsvl.utils import transform_to_matrix
import os
import math
import time
import random
import numpy as np
from PIL import Image
import sys


BASE_PATH = 'C:\\Users\\User\\Desktop\\Projects\\lgsvl-waymo-motion-scenario-gen\\dataset'
CALIB_PATH = BASE_PATH + "\\calib\\"
IMAGE_JPG_PATH = BASE_PATH + "\\image_jpg\\"
IMAGE_PNG_PATH = BASE_PATH + "\\image_2\\"
LIDAR_PCD_PATH = BASE_PATH + "\\velodyne_pcd\\"
LIDAR_BIN_PATH = BASE_PATH + "\\velodyne\\"


class KittiParser:
    def __init__(self, ego, sim, agent_name="Jaguar2015XE", start_idx=0):
        self.agent_name = agent_name
        self.sim = sim
        self.ego = ego
        self.ego_state = None
        self.sensor_camera = None
        self.sensor_lidar = None
        self.sensor_imu = None
        self.idx = start_idx

        # Sensor Calibrations: intrinsic & extrinsic
        self.camera_intrinsics = None
        self.projection_matrix = None
        self.rectification_matrix = None
        self.tr_velo_to_cam = None
        self.tr_imu_to_velo = None

# Sets up the required folder hierarchy and starts the simulator
    def bootstrap(self):
        os.makedirs(CALIB_PATH, exist_ok=True)
        os.makedirs(IMAGE_JPG_PATH, exist_ok=True)
        os.makedirs(IMAGE_PNG_PATH, exist_ok=True)
        os.makedirs(LIDAR_PCD_PATH, exist_ok=True)
        os.makedirs(LIDAR_BIN_PATH, exist_ok=True)

        self.load_sensors()
        self.calibrate()

        print("\nBootstrap success!")

# Saves the sensor objects for later use
    def load_sensors(self):
        print("\nAvailable sensors:")
        for sensor in self.ego.get_sensors():
            print("{}: {}".format(sensor.name, sensor.transform))
            if sensor.name == "Main Camera":
                self.sensor_camera = sensor
            if sensor.name == "Lidar":
                self.sensor_lidar = sensor
            if sensor.name == "IMU":
                self.sensor_imu = sensor

# Moves the EGO to the given transform
    def position_ego(self, transform):
        ego_state = self.ego.state
        ego_state.transform = transform
        self.ego.state = ego_state
        # cache the state for later queries
        self.ego_state = ego_state

# Saves camera, lidar, ground truth, and calibration data
    def capture_data(self):

        self.save_camera_image()
        self.save_lidar_point()
        self.save_calibration()

        self.idx += 1

# Saves a camera image from the EGO main camera as a png
    def save_camera_image(self):
        if self.sensor_camera:
            t0 = time.time()
            out_file = os.path.join(IMAGE_JPG_PATH, self.get_filename("jpg"))
            self.sensor_camera.save(out_file, quality=100)
            im = Image.open(out_file)
            png_file = os.path.join(IMAGE_PNG_PATH, self.get_filename("png"))
            im.save(png_file)
            print("{} ({:.3f} s)".format(out_file, time.time() - t0))
        else:
            print("Warn: Camera sensor is not available")

# Saves a LIDAR scan from the EGO as a bin
    def save_lidar_point(self):
        if self.sensor_lidar:
            t0 = time.time()
            pcd_file = os.path.join(LIDAR_PCD_PATH, self.get_filename("pcd"))
            self.sensor_lidar.save(pcd_file)
            # with open(pcd_file, "rb") as f:
            #     pc = self.parse_pcd_file(f)
            # bin_file = os.path.join(LIDAR_BIN_PATH, self.get_filename("bin"))
            # pc.tofile(bin_file)
            # print("{} ({:.3f} s)".format(bin_file, time.time() - t0))
        else:
            print("Warn: Lidar sensor is not available")

# Converts the lidar PCD to binary which is required for KITTI
    def parse_pcd_file(self, pcd_file):
        header = {}
        while True:
            ln = pcd_file.readline().strip()
            field = ln.decode('ascii').split(' ', 1)
            header[field[0]] = field[1]
            if ln.startswith(b"DATA"):
                break

        dtype = np.dtype([
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.uint8),
        ])
        size = int(header['POINTS']) * dtype.itemsize
        buf = pcd_file.read(size)
        lst = np.frombuffer(buf, dtype).tolist()
        out = []
        for row in lst:
            out.append((row[0], row[1], row[2], row[3] / 255))
        pc = np.array(out).astype(np.float32)

        return pc

# Calculates the calibration values between various sensors
    def calibrate(self):
        if self.sensor_camera and self.sensor_lidar:
            self.camera_intrinsics, self.projection_matrix, self.rectification_matrix = self.get_camera_intrinsics(self.sensor_camera)

            # Coordinate systems
            # - Unity:    x: right,   y: up,    z: forward (left-handed)
            # - Kitti: (right-handed)
            #   - Camera:   x: right,   y: down,  z: forward
            #   - Velodyne: x: forward, y: left,  z: up
            #   - GPS/IMU:  x: forward, y: left,  z: up

            # Velodyne to Camera
            diff_x = self.sensor_lidar.transform.position.x - self.sensor_camera.transform.position.x
            diff_y = -(self.sensor_lidar.transform.position.y - self.sensor_camera.transform.position.y)
            diff_z = self.sensor_lidar.transform.position.z - self.sensor_camera.transform.position.z
            self.tr_velo_to_cam = np.array([0, -1, 0, diff_x, 0, 0, -1, diff_y, 1, 0, 0, diff_z])  # Rotation: x: 90, y: 0, z: 90

            # IMU to Camera
            diff_x = self.sensor_imu.transform.position.z - self.sensor_lidar.transform.position.z
            diff_y = -(self.sensor_imu.transform.position.x - self.sensor_lidar.transform.position.x)
            diff_z = self.sensor_imu.transform.position.y - self.sensor_lidar.transform.position.y
            self.tr_imu_to_velo = np.array([1, 0, 0, diff_x, 0, 1, 0, diff_y, 0, 0, 1, diff_z])  # Rotation: x: 0, y: 0, z: 0
        else:
            print("Warn: Sensors for calibration are not available!")

# Calculates various camera properties
    def get_camera_intrinsics(self, sensor_camera):
        image_width = sensor_camera.width
        image_height = sensor_camera.height
        aspect_ratio = image_width / image_height
        vertical_fov = sensor_camera.fov
        horizon_fov = 2 * math.degrees(math.atan(math.tan(math.radians(vertical_fov) / 2) * aspect_ratio))
        fx = image_width / (2 * math.tan(0.5 * math.radians(horizon_fov)))
        fy = image_height / (2 * math.tan(0.5 * math.radians(vertical_fov)))
        cx = image_width / 2
        cy = image_height / 2

        camera_info = {}
        camera_info["image_width"] = image_width
        camera_info["image_height"] = image_height
        camera_info["aspect_ratio"] = aspect_ratio
        camera_info["vertical_fov"] = vertical_fov
        camera_info["horizontal_fov"] = horizon_fov
        camera_info["fx"] = fx
        camera_info["fy"] = fy
        camera_info["cx"] = cx
        camera_info["cy"] = cy

        projection_matrix = [
            camera_info["fx"], 0.0, camera_info["cx"], 0.0,
            0.0, camera_info["fy"], camera_info["cy"], 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        rectification_matrix = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        return camera_info, projection_matrix, rectification_matrix

    def get_transform(self, parent_tf, child_tf):
        tf = np.dot(transform_to_matrix(child_tf), np.linalg.inv(transform_to_matrix(parent_tf)))
        tf = np.array(tf)
        tf[:, 3] = tf[3, :]
        tf = tf[:3, :]
        tf_flatten = tf.flatten()

        return tf_flatten

# Saves the sensor calibration data
    def save_calibration(self):
        t0 = time.time()
        if self.camera_intrinsics is None or self.tr_velo_to_cam is None or self.tr_imu_to_velo is None:
            self.calibrate()
        txt_file = os.path.join(CALIB_PATH, self.get_filename("txt"))
        with open(txt_file, "w") as f:
            f.write("P0: {}\n".format(" ".join(str(e) for e in self.projection_matrix)))
            f.write("P1: {}\n".format(" ".join(str(e) for e in self.projection_matrix)))
            f.write("P2: {}\n".format(" ".join(str(e) for e in self.projection_matrix)))
            f.write("P3: {}\n".format(" ".join(str(e) for e in self.projection_matrix)))
            f.write("R0_rect: {}\n".format(" ".join(str(e) for e in self.rectification_matrix)))
            f.write("Tr_velo_to_cam: {}\n".format(" ".join(str(e) for e in self.tr_velo_to_cam)))
            f.write("Tr_imu_to_velo: {}\n".format(" ".join(str(e) for e in self.tr_imu_to_velo)))
            print("{} ({:.3f} s)".format(txt_file, time.time() - t0))

# Returns the current filename given an extension
    def get_filename(self, ext):
        return "{:06d}.{}".format(self.idx, ext)

# Returns the dimensions of the given bounding box
    def get_dimension(self, bbox):
        dimension = bbox.size
        height = dimension.y
        width = dimension.x
        length = dimension.z

        return height, width, length
