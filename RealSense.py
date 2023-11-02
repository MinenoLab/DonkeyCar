import pyrealsense2 as rs
import numpy as np
import cv2



class RealSense:
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        # config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 6)
        self.cap = cv2.VideoCapture(4)


        pipeline_profile = self.pipeline.start(config)
        # depth_sensor = pipeline_profile.get_device().first_depth_sensor()
        # depth_sensor.set_option(rs.option.depth_units, 0.001)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=3000)
        depth_frame = frames.get_depth_frame()
        # color_frame = frames.get_color_frame()
        for _ in range(10):
            _, color_frame = self.cap.read()
        #_, color_frame = self.cap.read()

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        # color_image = np.asanyarray(color_frame.get_data())

        # Convert BGR image to RGB for plotting
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # return color_image, depth_image
        return color_frame, depth_image