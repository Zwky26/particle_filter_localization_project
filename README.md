# particle_filter_localization_project

* Names: David Wu, Zack Wang

* How you will initialize your particle cloud (initialize_particle_cloud)?
  *   Using a similar approach to Class 6's exericse, we reference the occupancy grid we get from the House map-file to designate the set of tiles where we can place a point(inside the house). We then devise a random sampling on those tiles to get n points inside the house, of form (x, y). We random sample again on a uniform distribution to get the angle and combine these to get points of form (x, y, theta). We then use a mapping functinon (like the for-loop in initialize_particle_cloud) that takes these points, of form (x, y, theta), and turn them into Pose messages. We publish the list of these particles to ParticleCloud. 

* How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model)?
  *   We subscribe to odometer, and for any received message we update the position of every particle in the list, using a map function. If we need to translate between Quaternion and Euclidian distances we use the transofrmation functions from tf.transformations.
* How you will compute the importance weights of each particle after receiving the robot's laser scan data?(update_particle_weights_with_measurement_model)?
  *   Similar to Class 5, we will calculate the sum of abs(actual - predicted). We subscribe to liDar to get the actual scans, and to get the hypothetical scan, the scan if the robot were in the predicted Pose given, we use nav_msg.s map info/other functions. We plan on comparing the 8 cardinal directions (NSEW, NE, NW...) but can change this to adjust for accuracy. 
* How you will normalize the particles' importance weights (normalize_particles) and resample the particles (resample_particles)?
  *  We will use a numpy/math/stat function that normalizes the tuple (pose, weight). With this normalized list, we use random_sample on the list of poses, with weights (list of weights). 
* How you will update the estimated pose of the robot (update_estimated_robot_pose)?
  *  From the set of resampled particles, we update the estimated pose of the robot to be the particle with the highest weight (least difference to the actual scan).
* How you will incorporate noise into your particle filter localization?
  * During the adjustment step where we move particles based on the odometer. We can define a normal distribution centered at the observed action, then sample it to get the increments for each particle. Ex if odometer tells us we moved forward with velocity 1, we make a normal distribution with mean 1, small sd, and update each particle according to a sample from the normal.  
* A brief timeline sketching out when you would like to have accomplished each of the components listed above.
  *   By Thursday morning: particle cloud initialized, normal distribution generator, normalizing weights, resampling particles
  *   By Friday evening: liDar scan is working, update particles based on motion
  *   By Sunday evening: comparing scans, getting weights
  *   By Wednesday: troubleshooting, fine tuning 
