import math

def reward_function(params):
    def dist_2_points(x1, x2, y1, y2):
        return abs(abs(x1 - x2) ** 2 + abs(y1 - y2) ** 2) ** 0.5

    def closest_2_racing_points_index(racing_coords, car_coords):
        distances = []
        for i in range(len(racing_coords)):
            distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                     y1=racing_coords[i][1], y2=car_coords[1])
            distances.append(distance)
        closest_index = distances.index(min(distances))
        distances_no_closest = distances.copy()
        distances_no_closest[closest_index] = 999
        second_closest_index = distances_no_closest.index(min(distances_no_closest))
        return [closest_index, second_closest_index]

    def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
        a = dist_2_points(x1=closest_coords[0], x2=second_closest_coords[0], y1=closest_coords[1], y2=second_closest_coords[1])
        b = dist_2_points(x1=car_coords[0], x2=closest_coords[0], y1=car_coords[1], y2=closest_coords[1])
        c = dist_2_points(x1=car_coords[0], x2=second_closest_coords[0], y1=car_coords[1], y2=second_closest_coords[1])
        try:
            distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) - (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
        except ZeroDivisionError:
            distance = b
        return distance

    def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):
        heading_vector = [math.cos(math.radians(heading)), math.sin(math.radians(heading))]
        new_car_coords = [car_coords[0] + heading_vector[0], car_coords[1] + heading_vector[1]]
        distance_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=closest_coords[0], y1=new_car_coords[1], y2=closest_coords[1])
        distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0], x2=second_closest_coords[0], y1=new_car_coords[1], y2=second_closest_coords[1])
        if distance_closest_coords_new <= distance_second_closest_coords_new:
            next_point_coords = closest_coords
            prev_point_coords = second_closest_coords
        else:
            next_point_coords = second_closest_coords
            prev_point_coords = closest_coords
        return [next_point_coords, prev_point_coords]

    def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):
        next_point, prev_point = next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading)
        track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
        track_direction = math.degrees(track_direction)
        direction_diff = abs(track_direction - heading)
        if direction_diff > 180:
            direction_diff = 360 - direction_diff
        return direction_diff

    def indexes_cyclical(start, end, array_len):
        start = int(start or 0)
        if end < start:
            end += array_len
        return [index % array_len for index in range(start, end)]

    def projected_time(first_index, closest_index, step_count, times_list):
        current_actual_time = (step_count - 1) / 15
        indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))
        current_expected_time = sum([times_list[i] for i in indexes_traveled])
        total_expected_time = sum(times_list)
        try:
            projected_time = (current_actual_time / current_expected_time) * total_expected_time
        except ZeroDivisionError:
            projected_time = 9999
        return projected_time

    # Unpack parameters
    all_wheels_on_track = params['all_wheels_on_track']
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    progress = params['progress']
    steps = params['steps']
    speed = params['speed']
    heading = params['heading']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    track_length = params['track_length']
    is_offtrack = params['is_offtrack']
    is_reversed = params['is_reversed']

    # Racing line data (Example data, replace with actual data for RL Speedway)
    racing_track = [[3.47566, 1.44702, 3.19782, 0.06365],
                    [3.2855, 1.46245, 4.0, 0.04769],
                    [3.10177, 1.47088, 4.0, 0.04598],
                    [2.85739, 1.4788, 4.0, 0.06113],
                    [2.58489, 1.48588, 4.0, 0.06815],
                    [2.30751, 1.49136, 4.0, 0.06936],
                    [2.02628, 1.49531, 4.0, 0.07031],
                    [1.74191, 1.49784, 4.0, 0.0711],
                    [1.45488, 1.49907, 4.0, 0.07176],
                    [1.16559, 1.49913, 4.0, 0.07232],
                    [0.87434, 1.49815, 4.0, 0.07281],
                    [0.5814, 1.49627, 4.0, 0.07324],
                    [0.28702, 1.49365, 4.0, 0.0736],
                    [-0.00856, 1.49043, 4.0, 0.0739],
                    [-0.30511, 1.48674, 4.0, 0.07414],
                    [-0.60797, 1.48267, 3.17555, 0.09538],
                    [-0.91088, 1.47911, 2.32104, 0.13051],
                    [-1.21386, 1.47645, 1.91597, 0.15814],
                    [-1.5169, 1.47507, 1.66742, 0.18175],
                    [-1.81997, 1.47538, 1.66742, 0.18176],
                    [-2.1229, 1.47777, 1.66742, 0.18168],
                    [-2.4255, 1.48267, 1.66742, 0.1815],
                    [-2.72765, 1.49048, 1.66742, 0.18127],
                    [-3.02929, 1.50164, 1.66742, 0.18102],
                    [-3.33036, 1.5166, 1.66742, 0.18078],
                    [-3.61783, 1.53494, 1.66742, 0.17275],
                    [-3.88461, 1.52853, 1.66742, 0.16004],
                    [-4.13153, 1.48426, 1.66742, 0.15044],
                    [-4.35595, 1.39141, 1.66742, 0.14565],
                    [-4.54742, 1.23881, 1.89677, 0.12909],
                    [-4.70833, 1.04131, 1.89677, 0.1343],
                    [-4.83175, 0.79981, 1.89677, 0.14299],
                    [-4.90154, 0.52319, 1.89677, 0.15041],
                    [-4.90629, 0.24797, 1.89677, 0.14512],
                    [-4.85296, -0.00752, 1.89677, 0.1376],
                    [-4.74949, -0.23764, 1.89677, 0.13302],
                    [-4.60035, -0.43873, 1.89677, 0.13199],
                    [-4.40625, -0.60488, 2.25289, 0.11341],
                    [-4.18023, -0.74176, 2.5576, 0.10331],
                    [-3.92852, -0.85206, 3.00905, 0.09133],
                    [-3.65697, -0.94034, 3.78941, 0.07535],
                    [-3.36623, -1.01436, 4.0, 0.075],
                    [-3.0716, -1.07372, 4.0, 0.07514],
                    [-2.77586, -1.11984, 3.96854, 0.07542],
                    [-2.47954, -1.15451, 3.96854, 0.07518],
                    [-2.18303, -1.17945, 3.96854, 0.07498],
                    [-1.88662, -1.19638, 3.96854, 0.07481],
                    [-1.59056, -1.20694, 3.96854, 0.07465],
                    [-1.29536, -1.21273, 3.6162, 0.08165],
                    [-1.00285, -1.21527, 3.15441, 0.09273],
                    [-0.71748, -1.21303, 2.83052, 0.10082],
                    [-0.4374, -1.20374, 2.79362, 0.10031],
                    [-0.16144, -1.18516, 2.79362, 0.099],
                    [0.11107, -1.15487, 2.79362, 0.09815],
                    [0.38023, -1.10975, 2.79362, 0.09769],
                    [0.64792, -1.0574, 2.79362, 0.09764],
                    [0.9152, -1.00692, 2.57996, 0.10543],
                    [1.18271, -0.96362, 2.21093, 0.12257],
                    [1.44995, -0.93283, 1.95552, 0.13756],
                    [1.71609, -0.91968, 1.76865, 0.15066],
                    [1.98023, -0.92911, 1.62049, 0.1631],
                    [2.24117, -0.96609, 1.5, 0.17569],
                    [2.49776, -1.03127, 1.5, 0.17649],
                    [2.74873, -1.12398, 1.5, 0.17837],
                    [2.99505, -1.23495, 1.5, 0.18011],
                    [3.24279, -1.36515, 1.5, 0.18658],
                    [3.49674, -1.47212, 1.5, 0.18371],
                    [3.75639, -1.54343, 1.5, 0.17951],
                    [4.01678, -1.56816, 1.5, 0.17437],
                    [4.26794, -1.53781, 1.5, 0.16866],
                    [4.49603, -1.44807, 1.5, 0.16341],
                    [4.6849, -1.29807, 1.5, 0.16079],
                    [4.81422, -1.08819, 1.93552, 0.12737],
                    [4.89934, -0.8456, 1.95161, 0.13173],
                    [4.94306, -0.57775, 1.95161, 0.13906],
                    [4.94964, -0.2913, 1.95161, 0.14682],
                    [4.92523, 0.00726, 1.95161, 0.15349],
                    [4.86828, 0.29376, 1.95161, 0.14968],
                    [4.77873, 0.55543, 1.95161, 0.14171],
                    [4.65716, 0.78922, 1.95161, 0.13502],
                    [4.50427, 0.99187, 1.95161, 0.13007],
                    [4.32111, 1.1588, 1.95161, 0.12698],
                    [4.11152, 1.28248, 2.13526, 0.11397],
                    [3.89164, 1.36662, 2.36714, 0.09946],
                    [3.67713, 1.41812, 2.69476, 0.08186]]

    # Closest racing points
    closest_index, second_closest_index = closest_2_racing_points_index(racing_track, [x, y])
    closest_coords = racing_track[closest_index]
    second_closest_coords = racing_track[second_closest_index]

    # Distance from the racing line
    distance_racing_line = dist_to_racing_line(closest_coords, second_closest_coords, [x, y])

    # Direction difference
    direction_diff = racing_direction_diff(closest_coords, second_closest_coords, [x, y], heading)

    # Determine reward
    reward = 1.0

    # Penalize for distance from racing line (quadratic)
    reward *= max(1e-3, 1 - (distance_racing_line / (track_width * 0.5)) ** 2)

    # Penalize for direction difference
    reward *= max(1e-3, 1 - (direction_diff / 50.0) ** 2)

    # Speed adaptation based on curvature
    optimal_speed = closest_coords[2]
    speed_diff = abs(speed - optimal_speed)
    reward *= max(1e-3, 1 - (speed_diff / optimal_speed) ** 2)

    # Smooth steering
    steering_angle = params['steering_angle']
    reward *= max(1e-3, 1 - (abs(steering_angle) / 30.0))

    # Consistent progression
    if steps > 0:
        reward += (progress / steps) * 100

    # Projected lap time
    first_racingpoint_index = closest_index if 'first_racingpoint_index' not in params else params['first_racingpoint_index']
    proj_time = projected_time(first_racingpoint_index, closest_index, steps, [point[3] for point in racing_track])
    optimal_time = sum([point[3] for point in racing_track])
    reward *= max(1e-3, 1 - (proj_time - optimal_time) / optimal_time)

    # Efficient step use
    if steps > 0:
        reward += 10 * (progress / steps)

    # All wheels on track
    if not all_wheels_on_track:
        reward = 1e-3

    # Off track penalty
    if is_offtrack:
        reward = 1e-3

    # Reversed penalty
    if is_reversed:
        reward = 1e-3

    return float(reward)