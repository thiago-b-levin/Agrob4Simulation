# Webots ROS2 Docker Environment

This Docker setup provides a complete development environment for Webots robotics simulation with ROS2 integration.

## Prerequisites

Install Docker and required dependencies:

```bash
sudo apt install git make curl
curl -sSL https://get.docker.com | sh && sudo usermod -aG docker $USER
```

**Note:** After installing Docker, you may need to log out and back in for the user group changes to take effect.

## Quick Start

### First Time Setup

Build and run the container (only needed once):

```bash
ROS_DISTRO=humble ROS_TESTING=0 WEBOTS_VERSION=2025a make build run exec
```

### Daily Usage

After the initial setup, simply attach to the existing container:

```bash
make exec
```

## Container Management

### Building the Image
```bash
make build
```

You can customize the build with environment variables:
```bash
ROS_DISTRO=jazzy ROS_TESTING=1 WEBOTS_VERSION=2025a make build
```

### Starting the Container
```bash
make run
```

### Entering the Container
```bash
make exec
```

### Stopping the Container
```bash
docker stop webots_ros2-container
```

### Removing the Container
```bash
docker rm webots_ros2-container
```

### Viewing Container Status
```bash
docker ps -a
```

## Available Build Arguments

- `ROS_DISTRO`: ROS2 distribution (default: `humble`)
- `ROS_TESTING`: Enable ROS testing repository (default: `0`)
- `WEBOTS_VERSION`: Webots version to install (default: `2025a`)

## Pre-installed Tools and Libraries

### ROS2 Packages
- `ros-$ROS_DISTRO-vision-msgs`
- `ros-$ROS_DISTRO-ros2-control`
- `ros-$ROS_DISTRO-ros2-controllers`
- `ros-$ROS_DISTRO-xacro`
- `ros-$ROS_DISTRO-webots-ros2`

### Python Libraries
- NumPy
- Matplotlib
- PyTorch (CPU version)
- OpenCV
- TensorBoard
- Stable Baselines3
- Gymnasium

### Development Tools
- Git
- Nano
- GDB (debugger)

## Adding Dependencies

### Method 1: Modify Dockerfile (Recommended)

For permanent dependencies that should be available to all users:

1. **APT packages**: Add to the appropriate `RUN apt-get install` section:
   ```dockerfile
   RUN apt-get update && apt-get install -y \
       your-package-name \
       another-package
   ```

2. **Python packages**: Add to the Python dependencies section:
   ```dockerfile
   RUN pip3 install --no-cache-dir \
       your-python-package
   ```

3. **Rebuild the container**:
   ```bash
   make build run
   ```

### Method 2: Install Temporarily

For testing or temporary dependencies:

1. **Enter the container**:
   ```bash
   make exec
   ```

2. **Install packages** (will be lost when container is recreated):
   ```bash
   # APT packages
   sudo apt update && sudo apt install -y package-name
   
   # Python packages
   pip3 install package-name
   ```

### Method 3: Create Custom Dockerfile

For project-specific dependencies, create a new Dockerfile that extends the base image:

```dockerfile
FROM webots_ros2-image

# Add your custom dependencies
RUN apt-get update && apt-get install -y \
    your-specific-package

RUN pip3 install --no-cache-dir \
    your-specific-python-package
```

## Useful Aliases

The container includes these pre-configured aliases:

- `cb`: Colcon build with symlink install and debug flags
- `teleop`: Launch teleop twist keyboard with stamped messages

## Troubleshooting

### Display Issues
If you encounter display problems, ensure:
- X11 forwarding is working: `echo $DISPLAY`
- Xauthority permissions are correct
- Your user is in the video group

### GPU Issues
The container automatically detects NVIDIA GPUs. If you have issues:
- Ensure NVIDIA Docker runtime is installed
- Check `nvidia-smi` works on the host
- Verify GPU access with `docker run --gpus all nvidia/cuda:11.0-base nvidia-smi`

### Permission Issues
If you encounter permission issues with mounted volumes:
- Check that the UID matches between host and container
- Ensure your user has access to the source directory

## Container Features

- **Persistent data**: Source code is mounted from `../src/` to `/cyberbotics/ros2_ws/src/`
- **GPU support**: Automatic NVIDIA GPU detection and passthrough
- **Display forwarding**: X11 forwarding for GUI applications
- **Hardware access**: Full access to `/dev` for hardware interfacing
- **Network**: Host networking for easy ROS2 communication
