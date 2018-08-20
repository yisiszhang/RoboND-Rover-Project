import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

# The following functions are to map the world map/angle back to the rover's coordinate
# in order to help making decision on steering.
# Define a function to map world space to rover space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) + (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) - (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix, ypix, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix - xpos) * scale
    ypix_translated = (ypix - ypos) * scale
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_rover(xpix, ypix, xpos, ypos, yaw, x_size, y_size, scale):
    # Apply translation back to rover's coordinate
    xpix_tran, ypix_tran = translate_pix(xpix, ypix, xpos, ypos, scale)
    # Apply rotation to rover's navigation direction
    xpix_rot, ypix_rot = rotate_pix(xpix_tran, ypix_tran, yaw)
    # Clip to the rover's x-y limits
    x_pix_rover = np.clip(np.int_(xpix_rot), 0, x_size - 1)
    y_pix_rover = np.clip(np.int_(ypix_rot), 0, y_size - 1)
    # Return the result
    return x_pix_rover, y_pix_rover

def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status: isstuck and/or isspin
        if Rover.counter > 0:	# a counter to check movement every N steps (N is set to 10)
	    # Add up displacements within N steps
            Rover.cumdist += np.sqrt((Rover.pos[0] - Rover.lastpos[0])**2 + (Rover.pos[1] - Rover.lastpos[1])**2)
	    # Add up steering angle within N steps
            Rover.cumsteer += np.abs(Rover.laststeer - Rover.steer)
	    # During the period to add up the values, status is considered unstuck/unspin
            isstuck = 'NO'
            isspin = 'NO'
        else:
	    # If the cumulative displacement very small, then isstuck determined
            if Rover.cumdist>0 and Rover.cumdist<0.01:
                isstuck = 'YES'
            else:
                isstuck = 'NO'
	    # If the cumulative steering angle unchanged (usually when is always -15 degrees),
	    # is considered spinning around
            if Rover.cumsteer == 0:
                isspin = 'YES'
            else:
                isspin = 'NO'
            Rover.cumdist = 0
        if isstuck == 'YES':
	    # If is stuck, go to the 'stop' mode steps
            Rover.mode = 'stop'
        # check if rock is in view, if so move towards rock    
        if Rover.sample_in_view:
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)
            if np.min(Rover.rock_dists)<10:
                Rover.brake = Rover.brake_set
                # if stop before rock detectable, accelerate
                if not Rover.near_sample and isstuck == 'YES':
                    Rover.brake = 0
                    Rover.throttle = 0
                    Rover.steer = -15
            elif Rover.vel<=0.2: # if too slow and far
                Rover.throttle = Rover.throttle_set
                Rover.brake = 0
            
        elif Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                # penalize navigated terrian
                # repetitively visited terrian is defined 200 above average
                navigated_worldmap = Rover.worldmap[:,:,2] > np.mean(Rover.worldmap[:,:,2]) + 200
                navigated_y_world, navigated_x_world = navigated_worldmap.nonzero()
                if len(navigated_y_world)>0:
                    navigated_x_rover, navigated_y_rover = pix_to_rover(navigated_x_world, navigated_y_world, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.img.shape[1], Rover.img.shape[0], 10)
                    navigated_dists, navigated_angles = to_polar_coords(navigated_x_rover, navigated_y_rover)
                    nav = np.column_stack((Rover.nav_angles,Rover.nav_dists))
                    avoid = np.column_stack((navigated_angles,navigated_dists))
                    # get navigable but visited angles
                    overlap = np.array([x for x in set(tuple(x) for x in nav) & set(tuple(x) for x in avoid)])
                    if overlap.shape[0]>0:
                        Rover.avoid_angle = np.mean(overlap[:,0] * 180/np.pi)
                    else:
                        Rover.avoid_angle = 0
                else:
                    Rover.avoid_angle = 0                
                penal_strength = 0.2 # how harsh to penalize navigated angles
                if isspin == 'NO':
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)-Rover.avoid_angle * penal_strength, -15, 15)
                else:
                    Rover.steer = np.random.uniform(-15,15,1)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'          
            
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    if np.abs(Rover.laststeer) != 15:
                        Rover.steer = 15 * np.sign(Rover.laststeer) # Could be more clever here about which way to turn
                        if Rover.laststeer == 0:
                            Rover.steer = -15
                    else:
                        Rover.steer = Rover.laststeer

                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # in case rover stuck but camera view open
                    if Rover.throttle == 0:
                        # Set throttle back to stored value
                        Rover.throttle = Rover.throttle_set
                        # Release the brake
                        Rover.brake = 0
                        # Set steer to mean angle
                        Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                        Rover.mode = 'forward'
                    elif Rover.throttle > 0:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = -15
                        Rover.mode = 'stop'    
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel < 0.1 and not Rover.picking_up:
        Rover.send_pickup = True
        
    return Rover

