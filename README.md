# Hierarchical tasks control  
This exercise was about implementing a hierarchical task control algorithm for a pioneer robot to satisfy a set of constraint.  
The proposed solution allows to satisfy constraints on wheel velocities (maximum angular velocity) and visibility constraint of a sphere placed in the environment, while reaching a desired position on the plane.  
\
This was done through a quadratic programming approach, using a QP solver to find the optimal control input for the robot.
This robot was controllable through 4 variables: the speed of the 2 wheels + tilt and pan of the mounted camera.  




https://github.com/user-attachments/assets/8a17fe22-858e-47ae-aef5-904911c555ab


# IDE Configuration for CMake packages (ROS 1 / ROS 2 / non-ROS)

A script to generate QtCreator and VS Code configuration file for CMake, ROS 1 and ROS 2 projects


## Usage

Just run the script from the folder where the `CMakeLists.txt` file is. It will identify whether it is a raw CMake, ROS 1 or ROS 2 package and pick the suitable build directory.

## Options

- `-c <path>` : to indicate the path to `CMakeLists.txt` if it is not the current folder (default `.`)
- `-b <path>` : build folder (relative to CMake file) to use (default `./build` for raw CMake projects or standard package build folder for ROS packages)
- `--clean` : deletes the build folder before creating it again (default False)

## Generated files

- for Qt Creator: `CMakeLists.txt.user`
- for VS Code: `.vscode/settings.json`

### Why the symbolic links

Symbolic links named `qtcreator` are present for retro-compatibility reasons as this tool was only for Qt Creator initially.
