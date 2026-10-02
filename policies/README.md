# policies/

Configuration files of the navigation and estimation stacks. The exploration policy itself, FRONTIER3D, is code: `missions/frontier.py`, shared by both embodiments.

| File | Loaded by | Content |
|---|---|---|
| `nav2/nav2_params.yaml` | `missions/wheeled_nav.sh` | The reference arm's Nav2 configuration: the predecessor's revised configuration with the one correction marked in the file |
| `nav2/nav2_params_v8.yaml` | `missions/wheeled_nav.sh` (`batch_wheeled.sh --nav2-params`) | The predecessor's first Nav2 configuration in its v8 form (the localizer's baseline settings) with the corrections marked in the file, on which the wheeled robot was first run |
| `nav2/nav2_params_v8_polygon.yaml` | `missions/wheeled_nav.sh` (`batch_wheeled.sh --nav2-params`) | Footprint sensitivity variant of v8: the robot's true outline in place of its two circles (the local costmap's 0.22 m radius and the collision monitor's 0.23 m stop zone) |
| `nav2/amcl_aerial.yaml` | `missions/aerial_amcl.sh` | AMCL + map_server on the frozen map, for the shared executive of both vehicles |
| `slam/slam_params.yaml` | `missions/aerial_slam.sh`, `missions/wheeled_nav.sh` | slam_toolbox (online_async) for the exploration phase |
| `odom/rf2o_aerial.yaml` | `missions/aerial_odom.sh` | rf2o lidar odometry |
| `openvins/*.yaml` | `--odom-source vio` only | OpenVINS mono configuration and the IMU / camera-IMU chains of the simulated sensors |
