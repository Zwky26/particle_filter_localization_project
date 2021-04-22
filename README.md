# particle_filter_localization_project

* Names: David Wu, Zack Wang

## Implementation Plan

* How you will initialize your particle cloud (initialize_particle_cloud)?
  *   Using a similar approach to Class 6's exericse, we reference the occupancy grid we get from the House map-file to designate the set of tiles where we can place a point(inside the house). We then devise a random sampling on those tiles to get n points inside the house, of form (x, y). We random sample again on a uniform distribution to get the angle and combine these to get poses of form (x, y, theta). We then use a mapping functinon (like the for-loop in initialize_particle_cloud) that takes these points, and turn them into Pose messages. We publish the list of these particles to ParticleCloud. 

* How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
  *   We subscribe to odometer, and for any received message we update the position of every particle in the list, using a map function. If we need to translate between Quaternion and Euclidian distances we use the transofrmation functions from tf.transformations. To test, we start with just a few particles, with simple movements of the robot and visually confirm. If movement does not pass a visual test, we can test the helper function with a set pose and message to see if calculations are off. 
* How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
  *   Similar to Class 5, we will calculate the sum of abs(actual - predicted). We subscribe to liDar to get the actual scans, and to get the hypothetical scan, the scan if the robot were in the predicted Pose given, we use nav_msg.s map info/other functions. We plan on comparing the 8 cardinal directions (NSEW, NE, NW...) but can change this to adjust for accuracy. To. test, we again start with just a few particles, printing out the calculated weights. 
* How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
  *  We will use a numpy/math/stat function that normalizes the tuple (pose, weight). With this normalized list, we use random_sample on the list of poses, with weights (list of weights). To test, we wil print out the before-and-after normalization weights. After normalization, the sum of weights should equal 1.
* How you will update the estimated pose of the robot (update_estimated_robot_pose)?
  *  From the set of resampled particles, we update the estimated pose of the robot to be the average of all the resampled particles. We can visually test this, as we publish to rViz the estimated pose as well. It should roughly converge onto the actual location of the Turtlebot.
* How you will incorporate noise into your particle filter localization?
  * During the adjustment step where we move particles based on the odometer. We can define a normal distribution centered at the observed action, then sample it to get the increments for each particle. Ex if odometer tells us we moved forward with velocity 1, we make a normal distribution with mean 1, small sd, and update each particle according to a sample from the normal.  
* A brief timeline sketching out when you would like to have accomplished each of the components listed above.
  *   By Thursday morning: particle cloud initialized, normal distribution generator, normalizing weights, resampling particles
  *   By Friday evening: liDar scan is working, update particles based on motion
  *   By Sunday evening: comparing scans, getting weights
  *   By Wednesday: troubleshooting, fine tuning 

## Writeup

* Goal: The goal of this project was the implement a particle filter, that given a map apriori, can use scans and movements of the Turtlebot to determine its location. This process of localization was done by implementing an algorithm that updated n discrete particle/poses, that eventually converged on the true position of the Turtlebot. 
* High Level Description
  *   Initialization of particle cloud: When we initialize the particle cloud, we use methods "get_map", "map_point_to_rviz_coord", "draw_random_sample", "initalize_particle_cloud", "fix_p", "normalize_particle", and "publish_particle_cloud". We store the map metadata with "get_map" for the House file and use its occupancy grid to determine which spaces the Turtlebot physically could be in. In "map_point_to_rviz_coord", we transform each of these possible cells into the (x, y) coordinate representation used by RViz. In "initalize_particle_cloud", we uniformly draw from these points to get our particles using "draw_random_sample". We then iterate over the list of particles, for each generating a random angle and converting to quarternion representation. We finally normalize the particle weights with "normalize_particles" and publish them, allowing RViz to visualize the filter.  
  * Movement model: Movement model primarily uses "update_particles_with_motion_model", which is called within "robot_scan_received". If the Turtlebot has moved enough to perform an update, we calculate the theoretical distance from our old x, y, and yaw based on the motion reported by odom. For every particle in the filter, we update its x, y, and theta to reflect the change given, adding in some artificial noise with the "generate_noise" function. This draws on a normal distribution, with mean being the respective x, y, or yaw if there was absolutely no noise.  
  * Measurement model: 
  * Resampling
  * Incorporation of noise
  * Updating estimated robot pose
  * Optimization of parameters
   
* Challenges
* Future Work
* Takeaways
