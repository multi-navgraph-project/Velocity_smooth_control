# Velocity_smooth_control
Smoothing the angular and linear velocity output by the TEB to elliminate signal distortions.


**velocity_smoother: using the exponential approach or the exponential decay algorithms to smooth the speed signal based on the previous instances. A delay in the sognal would in increase with the smoothing intensity.


- Parameters


 twist_topic: topic to subscribe for the output velocity by the local planner.
 
 smooth_twist_topic: topic to publish the smooth velocity.
 
 averaging_method: 1 for exponential decay algorithm and 2 for exponential approach algorithm.
 
 smooth_with_zero: true to start the smooth signal from zero and false to tart from the initial value.

beta: a float between 0 and 1 to control the smoothing procedure. Approching 0 the signal would be slightly smoothed, approching 1 the signal would be very smooth (at exact 1.0 the exponential decay algorithm would ouput just the same signal but with a delay).
 
 
 
**twiste_to_joints: the parameter of the subscribed velocity topic is extracted (a launch file is created).
 
 
- Parameters


 twist_topic: topic to subscribe for the output velocity to convert.
