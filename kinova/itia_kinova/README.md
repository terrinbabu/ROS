# ITIA_KINOVA PACKAGE
The package has a set of functionalities for the use of the Jaco2 robot (j2n6s300) in simulation and in the real world.
It was tested under kinetic.

## HOW TO SETUP THE ENVIRONMENT
In order to setup the environment for moveit, moveit+gazebo and the real robot execute respectively:

```roslaunch itia_kinova kinova_j2n6s300 fake_contoller:=true```
```roslaunch itia_kinova kinova_j2n6s300 fake_contoller:=false gazebo_controller:=true```
```roslaunch itia_kinova kinova_j2n6s300 fake_contoller:=true gazebo_controller:=false```

## HOW TO TEST `itia::knutils::j2n6s330_handle`
In order to test the class `itia::knutils::j2n6s330_handle`, execute for moveit+gazebo and the real robot respectively:

```roslaunch itia_kinova kinova_move gazebo_controller:=true```
```roslaunch itia_kinova kinova_move gazebo_controller:=false```

TODO:

1. Have a look at the ikfast plugin that does not work (probably it was prepared under indigio and not kinetic)
2. The object loaded in moveit as to be loaded also in gazebo 
3. Sometimes the robot stops before in order to avoid some collisions – as you already told me sometimes collisions are not considered during planning. Since we are using gazebo, the robot stops before the collision happens. If this happens, we have the error “Got a callback on a goalHandle that we're not tracking.  This is an internal SimpleActionClient/ActionClient bug. This could also be a GoalID collision” and the warning “Dropping first 1 trajectory point(s) out of …., as they occur before the current time. First valid point will be reached in …...”. 

## HOW TO ?????????????
@Terrin. please create a readme file for the otehr launch file and specifically for the voice application and the pomdp application


## Contributors

 - Terrin Babu Pulikottil, @itia.cnr.it
 - Stefania Pellegrinelli, @itia.cnr.it
