#!/usr/bin/env python3
import cv2
import numpy as np
import pyrealsense2 as rs

def main():
    # 配置深度和彩色流
    pipeline = rs.pipeline()
    config = rs.config()

    # 使用默认的设备配置彩色和深度流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 开始流传输
    profile = pipeline.start(config)
    
    # 获取深度传感器和颜色传感器
    depth_sensor = profile.get_device().first_depth_sensor()
    color_sensor = profile.get_device().query_sensors()[1]

    # 创建窗口
    cv2.namedWindow('Depth')
    cv2.namedWindow('Color')

    # 如果深度传感器支持曝光设置，则创建滑动条
    if depth_sensor.supports(rs.option.exposure):
        def update_depth_exposure(value):
            depth_sensor.set_option(rs.option.exposure, value)
        cv2.createTrackbar('Depth Exposure', 'Depth', 1, 1600, update_depth_exposure)

    def update_color_exposure(value):
        color_sensor.set_option(rs.option.exposure, value)

    # 创建彩色图像的滑动条
    cv2.createTrackbar('Color Exposure', 'Color', 1, 1600, update_color_exposure)

    try:
        while True:
            # 等待一组帧：深度和彩色
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame or not color_frame:
                continue

            # 将图像转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # 应用伪色彩以更好地可视化深度图
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 显示图像
            cv2.imshow('Depth', depth_colormap)
            cv2.imshow('Color', color_image)

            # 按 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 停止流传输
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
