# stretrch-rtabmap
Setup guide for RTAB-Map on Stretch D435i robot. Integrates D435i camera for RGB-D odometry &amp; 3D mapping. Includes installation, configuration, and troubleshooting tips.

[![Video](https://img.youtube.com/vi/HGohfkw_41I/maxresdefault.jpg)](https://www.youtube.com/watch?v=HGohfkw_41I)


# RTAB-Map and RealSense D435i Camera Overview

RTAB-Map (Real-Time Appearance-Based Mapping) is an advanced SLAM (Simultaneous Localization and Mapping) system used in our Stretch Hello Robot RE2 setup. It is adept at creating detailed 3D maps and performing real-time localization. The RealSense D435i camera, integral to this setup, offers superior depth sensing, crucial for effective RGB-D odometry and SLAM. This combination provides the Stretch Robot with enhanced navigation and environmental mapping capabilities.
RTAB-Map Parameters for Stretch Hello Robot RE2

# Camera and Depth Topics:
        rgb_topic (value="/camera/color/image_raw"): The topic for the RGB images from the D435i camera.
        depth_topic (value="/camera/aligned_depth_to_color/image_raw"): The topic for depth images aligned with the color frame.
        camera_info_topic (value="/camera/color/camera_info"): Topic for RGB camera information, providing intrinsic parameters.
        depth_camera_info_topic (value="/camera/aligned_depth_to_color/camera_info"): Topic for depth camera information aligned to the color frame.

# Mapping and Odometry Parameters:
        odom_frame_id (type="string", value="vo"): Frame ID for odometry, indicating visual odometry reference.
        frame_id (type="string", value="base_link"): Specifies the robot's reference frame for mapping.
        Vis/CorType (value="1"): Sets feature-based correspondence for visual odometry.
        OdomF2M/MaxSize (type="string", value="1000"): Maximum size for feature-to-map odometry.
        Vis/MaxFeatures (type="string", value="600"): Maximum visual features to track.
        proj_max_height (value="2.0"): Maximum height for obstacle projection in the map.
        proj_max_ground_height (value="0.1"): Maximum height for ground projection.
        proj_max_ground_angle (value="10"): Maximum angle for ground projection.
        proj_min_cluster_size (value="10"): Minimum cluster size for point projection.

# Additional Parameters:
        args (value="--delete_db_on_start"): Resets the database at the start, ensuring a fresh mapping session.
        publish_tf_odom (default="true"): Determines whether to publish odometry transformations, set to true for continuous odometry tracking.

These parameters are tailored to harness the capabilities of the RealSense D435i camera for efficient SLAM and odometry with RTAB-Map on the Stretch Hello Robot RE2, balancing computational efficiency and mapping accuracy.