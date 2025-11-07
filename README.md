# Behavior Tree Examples and Common Nodes

This repository contains example Behavior Trees and custom Behavior Tree nodes built using [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

## Running Example Application
Clone this repository into your workspace

From the workspace directory:
```bash
vcs import src < src/behavior-tree-examples/dependencies.repos

colcon build

source install/setup.bash

ros2 run bt_application example_qt_gui
```

## Using Common Nodes in a Project
To use the common Behavior Tree nodes in a BT project, the .xml files (in the config folder) for the desired nodes package must be added under Models in the Groot2 editor.

Note: Some nodes may require additional dependencies to be installed. CMakeLists will look for these packages, and will include those nodes only if the dependencies are found. This will impact any nodes that depend on motoros2, PCL, the AIMS robotic grinding system, etc. Any nodes within an `#ifdef` block in the CMakeLists.txt and their .cpp file will be skipped if the dependencies are not found.