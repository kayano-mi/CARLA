import glob
import os
import sys
import time
import random
import time
import numpy as np
import cv2
import math

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation

IM_WIDTH = 640
IM_HEIIGHT = 480

actor_list = []


def process_semantic(image):
    image.convert(carla.ColorConverter.CityScapesPalette)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    cv2.imshow("", array)
    cv2.waitKey(20)
    print(array)
    return array / 255.0

def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((IM_HEIIGHT,IM_WIDTH,4))
    i3 = i2[:,:,:3]
    cv2.imshow("",i3)
    cv2.waitKey(10)
    return i3/255.0


def process_depth(image):
    image.convert(carla.ColorConverter.LogarithmicDepth)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    cv2.imshow("", array)
    cv2.waitKey(20)
    print(array)
    return array / 255.0

def rad_callback(radar_data):
    print(radar_data[0])
    velocity_range = 7.5  # m/s
    current_rot = radar_data.transform.rotation

    for detect in radar_data:
        #弧度转角度
        azi = math.degrees(detect.azimuth)
        alt = math.degrees(detect.altitude)
        # The 0.25 adjusts a bit the distance so the dots can be properly seen
        fw_vec = carla.Vector3D(x=detect.depth - 0.25)
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)

        def clamp(min_v, max_v, value):
            return max(min_v, min(value, max_v))

        norm_velocity = detect.velocity / velocity_range  # range [-1, 1]
        r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
        g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
        b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
        world.debug.draw_point(
            radar_data.transform.location + fw_vec,
            size=0.075,
            life_time=0.06,
            persistent_lines=False,
            color=carla.Color(r, g, b))


# 连接master
try:
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter("model3")[0]

    # spawn_point = Transform(Location(x=143.47872, y=-180.071747, z=5.5), Rotation(pitch=0, yaw=0, roll=0.0))
    # Location(x=136.478729, y=-194.071747, z=0.001673)
    # Actor(id=65, type=traffic.traffic_light)

    spawn_point = Transform(Location(x=25.4, y=22, z=1.5), Rotation(pitch=0, yaw=180, roll=0.0))
    vehicle = world.spawn_actor(bp, spawn_point)
    vehicle.set_autopilot(enabled=False)
    actor_list.append(vehicle)

    spawn_point1 = Transform(Location(x=10.4, y=22, z=1.5), Rotation(pitch=0, yaw=180, roll=0.0))
    vehicle1 = world.spawn_actor(bp, spawn_point1)
    vehicle1.set_autopilot(enabled=False)
    actor_list.append(vehicle1)


    # # Semantic camera
    # sem_cam = None
    # sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    # sem_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    # sem_bp.set_attribute("image_size_y", f"{IM_HEIIGHT}")
    # sem_bp.set_attribute("fov", str(105))
    # sem_location = carla.Location(0, 0, 30)
    # sem_rotation = carla.Rotation(-35, 0, 0)
    # sem_transform = carla.Transform(sem_location, sem_rotation)
    # sem_cam = world.spawn_actor(sem_bp, sem_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    # actor_list.append(sem_cam)
    # # 监听相机，并显示图像
    # sem_cam.listen(lambda image: process_semantic(image))

    # # RGB camera
    # # as the same use find method to find a sensor actor.
    # cam_bp = blueprint_library.find("sensor.camera.rgb")
    # # set the attribute of camera
    # cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    # cam_bp.set_attribute("image_size_y", f"{IM_HEIIGHT}")
    # cam_bp.set_attribute("fov", "110")
    # # add camera sensor to the vehicle
    # spawn_point = carla.Transform(carla.Location(0, 0, 30), carla.Rotation(-35, 0, 0))
    # sensor = world.spawn_actor(cam_bp, spawn_point, attach_to=vehicle)
    # actor_list.append(sensor)
    # sensor.listen(lambda data: process_img(data))

    # # Depth camera
    # depth_cam = None
    # depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
    # depth_location = carla.Location(0, 0, 30)
    # depth_rotation = carla.Rotation(-35, 0, 0)
    # depth_transform = carla.Transform(depth_location, depth_rotation)
    # depth_cam = world.spawn_actor(depth_bp, depth_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    # actor_list.append(depth_cam)
    # # 监听相机，并显示图像
    # depth_cam.listen(lambda image: process_depth(image))


    # # lidar
    # lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
    # lidar_bp.set_attribute('channels', str(128))
    # lidar_bp.set_attribute('points_per_second', str(9000000))
    # lidar_bp.set_attribute('rotation_frequency', str(20))
    # lidar_bp.set_attribute('range', str(60))
    # lidar_location = carla.Location(0, 0, 2)
    # lidar_rotation = carla.Rotation(0, 0, 0)
    # lidar_transform = carla.Transform(lidar_location, lidar_rotation)
    # lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)
    # lidar.listen(lambda point_cloud: \
    #                  point_cloud.save_to_disk(os.path.join('tutorial', '%06d.ply' % point_cloud.frame)))


    # # Semantic lidar
    # Semantic_lidar = blueprint_library.find('sensor.lidar.ray_cast_semantic')
    # Semantic_lidar.set_attribute('channels', str(128))
    # Semantic_lidar.set_attribute('points_per_second', str(9000000))
    # Semantic_lidar.set_attribute('rotation_frequency', str(20))
    # Semantic_lidar.set_attribute('range', str(60))
    # Semantic_lidar_location = carla.Location(0, 0, 2)
    # Semantic_lidar_rotation = carla.Rotation(0, 0, 0)
    # Semantic_lidar_transform = carla.Transform(Semantic_lidar_location, Semantic_lidar_rotation)
    # Semantic_lidar = world.spawn_actor(Semantic_lidar, Semantic_lidar_transform, attach_to=vehicle)
    # Semantic_lidar.listen(lambda point_cloud: \
    #                  point_cloud.save_to_disk(os.path.join('tutorial', '%06d.ply' % point_cloud.frame)))


    # # Radar
    # rad_cam = None
    # rad_bp = world.get_blueprint_library().find('sensor.other.radar')
    # rad_bp.set_attribute('horizontal_fov', str(35))
    # rad_bp.set_attribute('vertical_fov', str(20))
    # rad_bp.set_attribute('range', str(20))
    # rad_location = carla.Location(x=2.0, z=1.0)
    # rad_rotation = carla.Rotation(pitch=5)
    # rad_transform = carla.Transform(rad_location, rad_rotation)
    # rad_ego = world.spawn_actor(rad_bp, rad_transform, attach_to=vehicle,
    #                             attachment_type=carla.AttachmentType.Rigid)
    # rad_ego.listen(lambda radar_data: rad_callback(radar_data))
    #


    # # 观察者视角
    # spectator = world.get_spectator()
    # transform = vehicle.get_transform()
    # spectator.set_transform(carla.Transform(transform.location + carla.Location(z=20),
    #                                         carla.Rotation(pitch=-90)))


    time.sleep(10)


finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
