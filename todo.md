# Task plan
## Adaptive Navigator Layer
### Feature estimator
* Based on sensor measurements, compute the features, and send them to the adaptive navigator 
  * This means the feature estimator needs to ask the robots what measurements they're getting
  * Then I need to be able to pipe those sensor measurements to the Feature Estimator 
  * Then the Feature Estimator needs to be able use those measurements to compute a gradient direction and other features
  * Then I need to be able to pipe those features to the adaptive navigator 

### Adaptive navigator
* How the adaptive navigator transforms the features into a commanded velocity depends on the navigator's policy
  * That means I need to be able to tell the navigator what AN policy I want.
  * That means the policy influences the ultimate commanded velocity. The policies are listed in Kitts' paper.
  * This means the features have to be piped to the adaptive navigator.
  * This means the adaptive navigator will have to continously receive the features from the feature estimator.
  * This means I have to pipe the commanded velocity to the cluster controller.

## Cluster Space Controller
### Controller
* The controller has to convert the commanded cluster velocity from the adaptive navigator to individual robot velocities.
  * This means I have to receive input from the adaptive navigator
  * This means I have to numerically convert commanded cluster velocity into individual robot velocities
  * This means I have to pipe the robot velocities to each of the robots
* (optional) The controller has to use closed-loop control to make sure that desired velocity matches the actual.
  
## Robot Cluster
  * The robots have to travel according to their commanded velocities; additionally, the robots have to take sensor readings. The robots report their current velocity, positions, and sensor readings back to the Cluster Space Controller
    * This means that each robot has to receive input from the Controller
    * This means that the robot has to read the scalar field at the point they're located at.
      * This means that the SimulationField has to know where the robots are;
      * This means that the Simulation field has to use the robots' position to index the sensor readings, and return those sensor readings to the robots.
    * This means that each robot has to pipe its position, its velocity, and its sensor readings to the Cluster Space Controller.
    * This means that each robot has to move itself at each time step
      * The SimulationField has to know when a robot has moved
      * The SimulationField has to fetch the new robot positions

## Simulation Field
* Moving robots need to be seen moving across a scalar field toward the feature.
  * This means that the simulation field needs to be able to spawn a unique feature on the screen
  * This means the simulation field needs to spawn the map itself
  * This means the simulation field needs to be able spawn robots on the screen.
  * This means the field needs to know what the new positions of the robots are and graph them at every time step
  * The simulation needs to know where the robots are.
  * When the robots ask what their sensors are reading, the simulation field needs to tell them what the reading is based on where the robots are