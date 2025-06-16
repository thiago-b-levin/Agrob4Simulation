<div align="center">
  <p align="center">
    <img src="./docs/INESCTEC_MAIN.png" alt="Logotipo Instituição A" width="200"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
    <img src="./docs/TRIBE_MAIN.png" alt="Logotipo Instituição B" width="200"/>
  </p>
</div>

# Agrob4Simulation

This repository contains the tools that allows the use of the Webots-ROS2 framework that enables synthetic dataset generation, SLAM solution testing and other navigation solutions. 

## Table of Contents
<!-- - [Version History](#version-history) -->
- [Roadmap](#roadmap)
- [Dependencies](#dependencies)
- [Installation Instructions](#installation-instructions)
- [Testing Methodology](#testing-methodology)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributors](#contributors)
- [License](#license) 
<!-- - [Acknowledgments](#acknowledgments) -->
<!-- - [Prototype Status](#prototype-status) -->
<!-- - [Hardware Documentation](#hardware-documentation) -->
## Version History 
Below is a list of versions released for this project. Each version has a link to its release notes or relevant documentation.

| Version                        | Status     | Release Date | Description                                  |
|--------------------------------|------------|--------------|----------------------------------------------|
| [v1.0.0](https://github.com/thiago-b-levin/Agrob4Simulation/tree/v1.0.0) | Active     | 2025-06-04   | &bull; Webots-ROS2 Docker <br> &bull; Webots-Packages for simulation launching |

<!-- ## Prototype Status <span style="color:blue"> -- TODO --</span>
Current status of the prototype (e.g., "In Development," "Testing Phase," or "Completed"). Briefly describe the main focus of the current phase or any known limitations. -->

---

## Roadmap 
The planned milestones and tasks for the project's development. Completed tasks are marked.

- [x] **Milestone 1**: Initial project setup 
   - [x] Create repository
   - [x] Set up development environment
   - [x] Add basic documentation
- [x] **Milestone 2**: Core functionalities
   - [x] Launch Simulation with specific world 
   - [x] Spawn the robot in the simulation  
   - [x] Get sensor data and publish it
- [ ] **Milestone 3**: Testing
   - [x] Generate Synthetic Datasets
   - [x] Test SLAM Solutions
   - [x] Test Navigation Solutions 
   - [ ] Resolve known bugs
- [ ] **Milestone 4**: Final release preparations
   - [ ] Write release notes
   - [ ] Final documentation update

---

### Dependencies


Firstly, install the dependencies required to run the project:

- NVIDIA Container Toolkit

   ```bash
   sudo apt install -y nvidia-container-toolkit
   ```

- Docker

   Follow the instructions from this link: [Docker Installation Guide](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
   (In case you have docker installed, you can skip this step)

---

### Installation Instructions

Once all dependencies are installed, follow these steps to set up the project in your local environment.


<!-- 1. **Download the Docker**:

The following link will redirect to the page with the file: [TRIBE_SIM_DOCKER](https://inesctecpt-my.sharepoint.com/:u:/g/personal/jose_m_sarmento_office365_inesctec_pt/EbUW-F7KatlBmDAtwZaJkHMBNtD5apcV-E-TFoA7vFrxWA)

2. **Navigate to the project directory**:
```bash
cd folder_with_docker
```
3. **Give Sudo Permissions to Docker**:
```bash
sudo usermod -aG docker $USER
```
4. **Unzip the DockerFile**:
```bash
unzip trib_sim_exe.zip
```
5. **Load Docker**:
```bash
docker load -i trib_sim_exe.tar
```
If the docker was successfully installed, when you run:
```bash
docker images
```
You should get the following output:

```
REPOSITORY                 TAG       IMAGE ID       CREATED       SIZE
tribe_simulation           latest    f92722c46180   5 weeks ago   5.17GB
``` -->


1. **Clone the repository**

   ```bash
   git clone https://github.com/thiago-b-levin/Agrob4Simulation.git
   ```

2. **Go to Docker Directory**
   ```bash
   cd Agrob4Simulation/docker
   ```
3. **Build and run the container (only needed once)**


   ```bash
   ROS_DISTRO=humble ROS_TESTING=0 WEBOTS_VERSION=2025a make build run exec
   ```

### Usage

1. **After the initial setup, you simply attach to the existing container**
   ```bash
   make exec
   ```
   
   You should get the following output:

   ```
   cyberbotics@user:~/ros2_ws$
   ```
<!-- 1. **Inside the docker setup your environment as follows**
   ```bash
   cd <path_to_repository>/scripts
   chmod +x setup_environment.sh
   ./setup_environment.sh
   ``` -->

2. **Compile Webots Packages**
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Open the docker in another terminal to spawn the robot**
   ```bash
   cd /Agrob4Simulation/docker
   make exec
   source install/setup.bash
   ``` 

4. **Launch the simulation `world` environment**
   ```
   └── webots_worlds/
      ├── launch/
      │   └── world_launch.py
      ├── worlds/
      │   └── {your_world}.wbt
      ├── CMakeList.txt
      └── package.xml
   ```
   To launch webots and open your world, add it inside `/worlds` directory and run:
      ```bash
      ros2 launch webots_worlds world_launch.py world:={your_world}.wbt
      ```

5. **Spawn your robot as follows**
   ```
   robot/
   └── {robot_name}_description/
      ├── launch/
      │   └── spawn_{robot_name}.py
      ├── meshes/
      │   └── component.stl
      ├── robot
      │   └── {robot_name}.urdf
      ├── CMakeList.txt
      └── package.xml
   ```
   To spawn the robot run:
   ```bash
   source install/setup.bash
   ros2  launch {robot_name}_description spawn_{robot_name}.py
   ``` 

6. **OPTIONAL**: Create a plugin to add functionalities to the `WebotsNode` that is a core ROS2 node that interfaces with Webots
   ```
   src/
   └── {plugin_pkg_name}/
      ├── config/
      │   └── params.yaml
      ├── include/
      │   └── {plugin_pkg_name}
      │       └── plugin.hpp  
      ├── launch
      ├── src
      │   └── plugin.cpp
      ├── CMakeList.txt
      ├── plugin.xml
      └── package.xml
   ```

   1. Webots Plugin Interface:
      - The plugins implement Webots `PluginInterface`, allowing them to be dynamically loaded into the simulation.
      - This interface provides access to Webots' internal simulation environment
      - The plugin executes at each simulation step, processing sensor data or sending control commands.
   2. Structure Explanation:
      - **`plugin.cpp`**: plugin script.
      - **`plugin.hpp`**: plugin library.
      - **`plugin.xml`**: Defines how Webots loads the plugin, specifying the class.
      - **`config/`**: Contains optional YAML file to define parameters.
      - **`launch/`**: Can Include launch files to launch the plugin.
   3. Define the plugin in the robot's URDF file:
      ```
      <webots>
         <plugin type="{plugin_namespace}::{plugin_class}"/>
      </webots>
      ```
   4. To launch a Webots Node on the launch file add:

         ```py
         from webots_ros2_driver.webots_controller import WebotsController

         # Webots Driver Node with parameters
         webots_driver = WebotsController(
            robot_name={name that of the robot the node will be connected},
            parameters=[
               {'robot_description': {path_to_robot_urdf}},
            ]
         )
         ```  
      This modular structure enables **flexible integration** between Webots and ROS 2.

### Project Structure 

Overview of the main folders and files in the repository to help users navigate the code.


| Folder                                      | Description                                                                |
|---------------------------------------------|----------------------------------------------------------------------------|
| `/docker`                                   | Contains Docker-related files and configurations.                          |
| `/webots_packages`                          | Main package directory for running the Webots simulation.                  |
| `/webots_packages/gt_pub`                   | Node responsible for publishing ground truth data.                         |
| `/webots_packages/robots`                   | Packages for spawning and controlling robots within the simulation.        |
| `/webots_packages/wb_sensor_publisher`      | Sensor plugins for publishing sensor data from Webots.                     |
| `/webots_packages/webots_worlds`            | Simulation world files and launchers for specific simulation scenarios.    |


---

## Contributors 
We thank the following team members for their contributions to this project:

| Name                        | Email                                                             |
|-----------------------------| ----------------------------------------------------------------- |
| Thiago Levin                | [thiago.levin@inesctec.pt](mailto:thiago.levin@inesctec.pt)       | 
| André Aguiar                | [andre.s.aguiar@inesctec.pt](andre.s.aguiar@inesctec.pt)          |
| José Sarmento               | [jose.m.sarmento@inesctec.pt](mailto:jose.m.sarmento@inesctec.pt) |
| Pedro Rodrigues             | [pedro.rodrigues@inesctec.pt](mailto:pedro.rodrigues@inesctec.pt) |
| João Tomás Castro           | [joao.t.castro@inesctec.pt](mailto:joao.t.castro@inesctec.pt)     |
| Luís Santos                 | [luis.c.santos@inesctec.pt](mailto:luis.c.santos@inesctec.pt)     |


## Acknowledgments
This prototype was funded and developed under the following projects:

- [SCORPION](https://scorpion-h2020.eu/) - SCORPION’s objective is to develop a safe and autonomous precision spraying tool integrated into a modular unmanned tractor (robotics platform) to increase spraying efficiency, while reducing human and animal exposure to pesticides, water usage and labour costs.
- [ROMOVI](https://www.inesctec.pt/en/projects/romovi#about) - The RoMoVi project main objective is the development of robotic components and a modular and extensible mobile platform, which will allow in the future to provide commercial solutions for hillside vineyards capable of autonomously executing operations of monitoring and logistics.
- [ROSIN](https://www.rosin-project.eu/) - ROS-Industrial aims to transfer value and the ease of application to industrial hardware, by developing new components, improving existing ones, but also by performing non-development work such as compiling usage and development guidelines or performing license audits.

<div align="center">
    <p align="center">
      <img src="https://scorpion-h2020.eu/wp-content/uploads/2021/03/logo-scorpion-simple.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://www.advid.pt/uploads/PROJETOS/projetos-logos/L7-P1-ROMOVI-Logo_01.png" width="150" style="margin-right: 50px;"/>
    </p>
</div>

<div align="center">
    <p align="center">
      <img src="https://repositorio.inesctec.pt/logos/compete.png" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/norte2020/portugal2020.svg" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/ue-feder_cor.jpg" width="150" style="margin-right: 50px;"/>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; <!-- Espaço entre as imagens -->
      <img src="https://repositorio.inesctec.pt/logos/logo_cores.jpg" width="150" style="margin-right: 50px;"/>
    </p>
</div>


## License

This project is licensed under the [Apache License 2.0](LICENSE).


---