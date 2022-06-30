# Swerve MPC
The following package contains the the interface to the OCS2 library for the swerve mobile base. 

This interface can also be used for the base combined with and 6 DOF arm. For this please look at the **mabi_swerve** repository.

## Launch
To launch the OCS2's dummy visualization tool
```
roslaunch swerve_mpc swerve_mpc.launch
```

The parameters can be modified in `config/mpc/task.info`

## Rviz
To set the goal pose in Rviz please use the **2d Nav Goal** tool

## Legs Configuration
To change the legs configuration the **BrakesService** can be called from the command line
```
rosservice call /swerve_base/brakes_service ""
``` 
This service uses a custom message definition. This definition can be found in the `swerve_msgs` package.

### Nota Bene
- the base configuration will only change if a path is published