def reward_function(params):
    # Read input parameters
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    speed = params['speed']
    steering_angle = params['steering_angle']
    progress = params['progress']
    steps = params['steps']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']

    # Set some constants for tuning the reward function
    MAX_SPEED = 4.0
    MIN_SPEED = 1.0
    MAX_STEERING_ANGLE = 30.0

    #Set some weight constants
    SPEED_REWARD_WEIGHT = 0.5
    STABILITY_REWARD_WEIGHT = 0.4
    TRACK_REWARD_WEIGHT = 0.3
    PROGRESS_REWARD_WEIGHT = 0.2
    WAYPOINT_REWARD_WEIGHT = 0.1

    # Calculate reward based on speed
    speed_reward = 1.0
    if speed < MIN_SPEED:
        speed_reward *= 0.5
    elif speed >= MAX_SPEED:
        speed_reward *= 1.0
    else:
        speed_reward *= speed / MAX_SPEED

    # Calculate reward based on stability (steering angle)
    stability_reward = 1.0 - abs(steering_angle) / MAX_STEERING_ANGLE

    # Calculate 3 markers that are at varying distances away from the center line
    marker_1 = 0.1 * track_width
    marker_2 = 0.25 * track_width
    marker_3 = 0.5 * track_width

    # Give higher reward if the car is closer to center line and vice versa
    if distance_from_center <= marker_1:
        track_reward = 1.0
    elif distance_from_center <= marker_2:
        track_reward = 0.5
    elif distance_from_center <= marker_3:
        track_reward = 0.1
    else:
        track_reward = 1e-3  # likely crashed/ close to off track

    # Calculate reward based on progress
    progress_reward = progress / steps

    # Calculate the distance to the next waypoint
    next_waypoint = waypoints[closest_waypoints[1]]
    distance_to_next_waypoint = abs(next_waypoint[0] - params['x'])

    # Calculate reward based on distance to the next waypoint
    waypoint_reward = 1.0 - distance_to_next_waypoint / track_width

    # Combine the rewards with weights
    reward = (SPEED_REWARD_WEIGHT * speed_reward) + (STABILITY_REWARD_WEIGHT * stability_reward) + \
             (TRACK_REWARD_WEIGHT * track_reward) + (PROGRESS_REWARD_WEIGHT * progress_reward) + \
             (WAYPOINT_REWARD_WEIGHT * waypoint_reward)


    return float(reward)
