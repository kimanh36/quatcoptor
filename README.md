
There are three main packages that compose this project, these are:

 1. mavros_off_board
 2. object_detector
 3. drone_controller

In the package *mavros_off_board* are the launch files, world file, description files (urdf, xacro, sdf) and basic scripts to control the aircraft. The package *object_detector*: will start the detection module of the system and track the landing platform .. Finally, *drone_controller* has the proportional and MPC controllers developed to land the vehicle based on the estimations made with the *object_detector* package.

## Setting up the project

1. [Installation and environment configuration.](/Installation.md)
2. [Usage and deployment](/Usage.md)

# quatcopter
