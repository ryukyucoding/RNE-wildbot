# wildbot_workspace

Workspace repo for developing and running ROS 2 Jazzy nodes on the Wildbot Car.

## Project Structure

```
wildbot_workspace/
  Dockerfile                          # Build your custom ROS 2 image
  launch_shell.sh                     # Build image + open interactive shell
  launch_ros2_package_example.sh      # Build image + run a single ROS 2 command
  docker/compose/
    .env                              # ROS_DOMAIN_ID (set per-user!)
    configs/
      hardware.yaml                   # Car hardware parameters
      controllers.yaml                # ros2_control controller config
    docker-compose_kros_car.yml       # Car bringup service
    docker-compose_camera_gemini.yml  # Depth camera service
    docker-compose_oradarlidar.yml    # LiDAR service
    docker-compose_lidar_pkg.yml      # LiDAR filter service
    docker-compose_rosbridge_server.yml # rosbridge server
  scripts/                            # Launch car peripherals via docker compose
    00_start_all.sh                   # Start all peripheral services
    kros_car.sh                       # Car only
    camera_gemini.sh                  # Camera only
    lidar.sh                          # LiDAR
    rosbridge_server.sh               # rosbridge server only
    utils.sh                          # Shared helper functions
  workspaces/src/                     # Put your own ROS 2 packages here
```

## Quick Start

### 1. Set your ROS_DOMAIN_ID

Edit `docker/compose/.env` and pick a unique ID (0-101) so you don't collide with other users on the same network:

```
ROS_DOMAIN_ID=42
```

### 2. Start the car peripherals

The `scripts/` folder launches the car's built-in hardware (motors, camera, LiDAR, etc.) via pre-built images. No build step needed.

```bash
# Start everything (car + camera + LiDAR + rosbridge)
./scripts/00_start_all.sh

# Or start individual services
./scripts/kros_car.sh
./scripts/camera_gemini.sh
./scripts/lidar.sh
./scripts/rosbridge_server.sh
```

Press `Ctrl+C` to stop all services launched by that script.

### 3. Run your own ROS 2 nodes

Place your ROS 2 packages in `workspaces/src/`, then use one of the two launch methods:

**Interactive shell** -- build the image, drop into a bash shell inside the container:

```bash
./launch_shell.sh
```

Inside the container you can build and run as usual:

```bash
cd /workspaces
colcon build --symlink-install
source install/setup.bash
ros2 run my_package my_node
```

**Direct command** -- copy `launch_ros2_package_example.sh`, change the final command, and run it:

```bash
cp launch_ros2_package_example.sh launch_my_node.sh
# Edit the command at the end of "docker run" into: ros2 run my_package my_node
./launch_my_node.sh
```

### 4. Install extra ROS 2 packages

If your nodes depend on packages not in the base image, add them in the `Dockerfile`:

```dockerfile
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-nav2-bringup \
        ros-${ROS_DISTRO}-slam-toolbox && \
    rm -rf /var/lib/apt/lists/*
```

Then run `./launch_shell.sh` (or your custom launch script) -- it detects Dockerfile changes and rebuilds automatically.

## Hardware Configuration

The car's hardware parameters are configured via YAML files in `docker/compose/configs/`.

The details of the controlling will be at README_kros_car.md.

## Notes

- The `scripts/` services use the pre-built `ghcr.io/screamlab/wildbot_base_image:latest` image. Your custom nodes use the locally built `wildbot_workspace` image from the `Dockerfile`.
- `workspaces/src/` is bind-mounted into the container at `/workspaces/src`, so code changes are reflected immediately (with `--symlink-install`).
- `docker/compose/configs/` is mounted read-only at `/configs` -- edit configs on the host, restart the service to apply.
- `enable_odom_tf: false` in the base controller means no `odom -> base_link` TF is published. Your localization stack should provide that transform.
- Camera's resolution can be changed; but beware of the network bandwith, using compressed image is definitely recommended.
- LiDAR publishes it's unfiltered data in `/scan_tmp`, while we filtered out bad data and publishes into `/scan` using our self developed `lidar_pkg` node. 



## Tables

### Camera's supported resolution & framerates

Image:

| **Resolution** | **FPS** |
| --- | --- |
| **1920x1080** | 30 |
| **1280x720** | 30 |
| **640x480** | 60, 30, 15, 10, 5 |

Depth:
| **Resolution** | **FPS** |
| --- | --- |
| **1280x800** | 30, 15 |
| **640x400** | 60, 30, 15, 10, 5 |
| **320x200** | 60, 30, 15, 10, 5 |
