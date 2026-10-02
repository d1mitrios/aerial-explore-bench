# policies/

Configuration files of the navigation and estimation stacks. The exploration policy itself, FRONTIER3D, is code: `missions/frontier.py`, shared by both embodiments.

| File | Loaded by | Content |
|---|---|---|
| `nav2/nav2_params.yaml` | `missions/wheeled_nav.sh` | The predecessor's Nav2 configuration (v9) with the corrections marked in the file |
| `nav2/nav2_params_polygon.yaml` | `missions/wheeled_nav.sh` (`batch_wheeled.sh --nav2-params`) | Footprint sensitivity variant: the robot's true outline in place of the 0.22 m circle |
| `nav2/amcl_aerial.yaml` | `missions/aerial_amcl.sh` | AMCL + map_server on the frozen map, for the shared executive of both vehicles |
| `slam/slam_params.yaml` | `missions/aerial_slam.sh`, `missions/wheeled_nav.sh` | slam_toolbox (online_async) for the exploration phase |
| `odom/rf2o_aerial.yaml` | `missions/aerial_odom.sh` | rf2o lidar odometry |
| `openvins/*.yaml` | `--odom-source vio` only | OpenVINS mono configuration and the IMU / camera-IMU chains of the simulated sensors |
