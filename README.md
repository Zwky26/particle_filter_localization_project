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
  * Movement model: Movement model primarily uses "update_particles_with_motion_model", which is called within "robot_scan_received". If the Turtlebot has moved enough to perform an update, we calculate the theoretical distance from our old x, y, and yaw based on the motion reported by odom. For every particle in the cloud, we update its x, y, and theta to reflect the change given, adding in some artificial noise with the "generate_noise" function. This draws on a normal distribution, with mean being the respective x, y, or yaw if there was absolutely no noise.  
  * Measurement model: For measurement model, we primarily use the method "update_particle_weights_with_measurement_model", which is called within "robot_scan_received" , right after "update_particles_with_motion_model". We fetch the scan measurements for the cardinal directions (North, Northwest... Northeast). For every particle, we consider every measurement that is not infinite (indicating there is some object within sight). We use the ray casting projection from class to identify the hypothetical rays for each angle, and consult the Likelihood Field data to get a distance. Using "compute_prob_zero_centered_gaussian", we convert this distance into a probability, which is multiplied for each angle and assigned as the weight to the particle.    
  * Resampling: Resampling occurs in the "resample_particles" method, which is called after "update_particle_weights_with_measurement_model". Before we do so though, we call "normalize_particles", which scales all the particle weights so that they sum to one. With this normalized list of particles, we use the same "draw_random_sample" function from initialization with the updated weights, saving the new particle cloud.
  * Incorporation of noise: Noise is added with the "generate_noise" method. It is called during the movement/motion model step. Using the numpy.random package, we take the current coordinate, the theoretical difference reported by odom, and output a new coordinate that may differ slightly from the theoretical coordinate. We do this for every point, for their x, y, and theta values.  
  * Updating estimated robot pose: This step is done in "update_estimated_robot_pose". We iterate throguh all the particles in the particle cloud, calculating the average x, y, and quarternion representation. We convert this average to a pose, and save the new pose.  
  * Optimization of parameters: The main parameters that we altered were the number of particles in the cloud and the standard deviation for the measurement model step. Originally we had 20,000 particles, as we thought having more would better ensure a match to the actual pose. However, after troubleshooting with Pouya, we learned we had too many particles, and it was having a counterproductive effect on the filter. We lowered it to 2000, which solved most of the problems immediately. With the standard deviation for the measurement step, we lowered the standard deivation from 0.5 to 0.4, eventually to 0.1. This was done eyeballing how "tolerant" the selection process. Our choice to make the standard deviation of the noise generator 0.5 was because we thought it was a good balance between having the noise be significant enough, but not too jarring.
   
## Other Deliverables
   
* Challenges: There were two main stopping points for the project. After importing some of the code from Class 6, we found that RViz what not publishing any particles. Using "rostopic echo particlecloud", no particles were being published. After getting help in the Slack channel, we found that the starter code from Class 6 uses topic "particlecloud" whereas particle_filter uses the topic "particle_cloud". After fixing this discrepency, the particles were being published, but not visualized. After some trial-and-error, we discovered that the particles were being visualized, just very far off the map. Using the map metadata, this problem was solved quickly by scaling the occupancy grid values to x,y coordinates. The other significant problem weencountered was particles dying off. After one or two iterations of updating, all particles would disappear. We would also get the error of 'odom' timing out. After talking with Pouya during class worktime, we learned that we actually had too many particles. This was causing each of the particle weights to be very small, so small that given the random sampling step of the measurement model step, every point had a significant risk of being thrown out, regardless of its "correctness". To solve this, we lowered the number of particles and made the measurment model step more lenient. This solved the issue, and the filter was more or less functional after this fix. 
* Future Work: The most immediate addition we could do is add a random error term to the measurement model step. This was optional for the scope of the project, but adding a (z-random / z-max) term would make the model more adaptive to a physical version. Something else that could be improved is optimality. Several of the methods iterate through the entire particle cloud to fetch data, which is fine for the scope of the project, but if the Turtlebot were to go significantly faster, the strain of such methods might cause the program to crash. We saw this with an odom timeout when the program still had 20,000 particles. We would also like to optimiaze our code to potentially allow for more particles.
* Takeaways: 
    * The biggest takeaway seemed to be how problems can be approached with less traditional methods. One problem, with the particles not showing up, could have been quickly solved if you zoomed out a bit. All the particles were just translated over by 10 units. For the other major problem, with particles dying out too quickly, raising the number of particles did not fix the solution. Although we eventually received help from Pouya, the lesson learned is that we should've tried lowering the number of particles. Even if that seemed unintuitive to our hypotheses at the time, experimentation is worth trying, especially for the more non-deterministic problems that will arise. 
    * A second takeaway from this project is how useful it was to discretize the work. Establishing a timeline for parts of implementation allowed for clearer goals and a more evenly (and front-loaded) programming experience. For future projects, keeping this in mind will help in the event that there are less defined objectives or methods, especially when working as a group.  


*Gif (estimated pose colored blue for emphasis): ![gif](https://github.com/Zwky26/particle_filter_localization_project/blob/main/particle.gif)
