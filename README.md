# Robot Localization

## Environment

For the environment I have chosen Gazebo simulation, where I made up my own room in which a robot must localize itself using landmarks. These landmarks were distributed in the corners of the room and represented by cubes or spheres. The position of the landmarks was given to the robot in advance.

## Robot
For the robot I decided to go with Tiago robot ([PAL Robotics](https://pal-robotics.com/robots/tiago/) – Titanium). It is a two wheeled robot, which also has an arm, but this was neglected for the purpose of this project. Furthermore, Tiago has a laser sensor, an IMU, as well as a RGB and depth camera.

## Sensor model
For the distance from the robot to the different landmarks I used the Euclidean norm. The sensor model consists of an artificial GPS which sends noisy estimation of the distance to the landmarks.

## Movement
The robot was able to move straight forward and was able to rotate, either in the left or in the right direction.

## Motion model
As the movement of the robot was given in distance units and turn units (radiant), the robots position was set accordingly. First, the orientation of the robot was set, by adding the desired turn in radiant to the current orientation. To limit the values of the orientation to be within the range -π to π, an orientation over π was projected to negative π plus the delta between π and the orientation, and vice versa. Secondly, the distance was set and new values were computed for x and y, respectively. This was done by using the unit circle, where `x + cos(angle) * distance` leads to the new x value and `y + sin(angle) * distance` to the new y value. The robot’s position was set to these new values only if they were valid.

## Localization module
For this project I wanted to implement a particle filter. To achieve this, I used multiple
classes:
* `Robot` class which included all the movement, and sensing functions, and as properties the estimated and true pose (x, y, orientation), the world settings (size, landmarks) and the GPS and sensing noise
* `Particle` class, which included movement and sensing as the robot, but without actually moving Tiago, and a weighting function, that weighted the particle according to the measurements (distance to landmarks).
* `ParticleFilter` class which was the parent class of the Particle class and included the world settings, an array of n particles which were initialized when a new ParticleFilter was created, the weights of the particles (initialized with 1/n for each particle) and functions to simulate the motion of all particles, weight all particles, resample particles based on their weight, estimate the robots position based on the particles, and lastly, visualize the current situation.
* `Main_localization` class that combined all classes and ran the localization
* `Util` class with helper functions, e.g. computing the distance, gaussian function, limiting function, conversion function from quaternions to Euler angles, function to compute the mean of circular quantities, a normalization function, and the `VelMsg` class which initialized a Twist message with all values being zero, and setter function for either the forward movement or rotation
* `World` class where the landmarks and the size of the world was stored


As a first step of localization, a world object was created using an array of landmarks, and the size of the world. Then a `ParticleFilter` object was created, which had as arguments the number of particles, the noise for moving a particle forward, turning it, or for its sensing, and the world object. Thereafter a `rospy` node and the robot were initialized and the loop for the particle filter was started. In each iteration, the robot was moved, either randomly or with a fixed pattern (both can be seen in the video). Then the robots motion was simulated for each particle. This was achieved in the same manner as the robot motion was achieved, but without moving Tiago.

In a second step the particles got weighted according to the fitting of them to the actual measurements of the robot, and then normalized, so they sum up to one in the end. The third step was to resample the particles, but with higher weighted particles being drawn more often than particles with less weight. The final step was the estimation of the robots position based on the particles. I used the weighted mean for all particles to estimate x and y, and the [mean of circular quantities](https://en.wikipedia.org/wiki/Mean_of_circular_quantities) for the orientation.

## Results
In the beginning the particles were distributed uniformly across the map. ![Randomly distributed particles](./data/10trials/figure_0.png?raw=True =200x200).
With further movement the particles converged, as particles with higher weights were drawn more often. In the figure above one can see that yellow particles were the initial particles, and the green particles were the ones that were resampled. The black square represents the true position of the robot, the cyan triangle displays the robots position based on the motion model, and the blue triangle represents the estimate of the position based on the particles. Red symbolized the landmarks that were given to the robot in advance.
With increasing time and therefore, more movement, the particles slowly converged to be closer to the true position of the robot, and also the orientation converged, as it can be seen in the figure on the bottom. Drawing more and more particles that are closer to the true position also led, of course, to a better estimate.
