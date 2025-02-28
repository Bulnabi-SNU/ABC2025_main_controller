ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:=depth_camera_link \
    rgb_topic:=/depth_camera/image_raw \
    depth_topic:=/depth_camera/depth/image_raw \
    camera_info_topic:=/depth_camera/camera_info \
    approx_sync:=true \
    qos:=2 \
    rviz:=true
