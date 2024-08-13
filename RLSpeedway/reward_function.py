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
    racing_track = [[6.76597, 2.90388, 4.0, 0.0478],
                    [6.63674, 2.918, 4.0, 0.0325],
                    [6.50751, 2.93214, 4.0, 0.0325],
                    [6.33688, 2.95081, 4.0, 0.04291],
                    [6.03704, 2.9837, 4.0, 0.07541],
                    [5.73721, 3.01667, 4.0, 0.07541],
                    [5.43738, 3.04972, 4.0, 0.07541],
                    [5.13756, 3.08285, 4.0, 0.07541],
                    [4.83775, 3.11605, 4.0, 0.07541],
                    [4.53794, 3.14924, 4.0, 0.07541],
                    [4.23813, 3.18241, 4.0, 0.07541],
                    [3.93831, 3.21555, 4.0, 0.07541],
                    [3.63849, 3.24867, 4.0, 0.07541],
                    [3.33867, 3.28177, 4.0, 0.07541],
                    [3.03912, 3.31394, 4.0, 0.07532],
                    [2.74041, 3.34329, 4.0, 0.07504],
                    [2.44312, 3.36795, 3.97886, 0.07497],
                    [2.14784, 3.38615, 3.722, 0.07949],
                    [1.85515, 3.3962, 3.50109, 0.08365],
                    [1.56566, 3.3965, 3.29981, 0.08773],
                    [1.27997, 3.38557, 2.72457, 0.10493],
                    [0.9987, 3.36201, 2.72457, 0.1036],
                    [0.7225, 3.32445, 2.72457, 0.10231],
                    [0.4521, 3.27148, 2.72457, 0.10113],
                    [0.1883, 3.20171, 2.72457, 0.10015],
                    [-0.06793, 3.11358, 2.72457, 0.09946],
                    [-0.31258, 2.99916, 3.1585, 0.08551],
                    [-0.54821, 2.86536, 3.88211, 0.0698],
                    [-0.77772, 2.7186, 4.0, 0.06811],
                    [-1.00404, 2.56512, 4.0, 0.06837],
                    [-1.23658, 2.41084, 3.844, 0.0726],
                    [-1.47176, 2.26061, 3.57474, 0.07807],
                    [-1.71041, 2.11629, 3.4041, 0.08193],
                    [-1.95317, 1.97983, 3.4041, 0.08181],
                    [-2.20057, 1.8532, 3.4041, 0.08164],
                    [-2.45305, 1.73845, 3.4041, 0.08147],
                    [-2.71104, 1.6378, 3.4041, 0.08135],
                    [-2.97494, 1.5536, 3.4041, 0.08137],
                    [-3.24483, 1.4878, 3.62294, 0.07668],
                    [-3.5196, 1.43856, 3.88203, 0.07191],
                    [-3.79834, 1.40403, 4.0, 0.07022],
                    [-4.0803, 1.3824, 4.0, 0.0707],
                    [-4.36484, 1.3718, 4.0, 0.07118],
                    [-4.65138, 1.37035, 3.9602, 0.07236],
                    [-4.93943, 1.37605, 3.50277, 0.08225],
                    [-5.2285, 1.38685, 3.15493, 0.09169],
                    [-5.50923, 1.39453, 2.88944, 0.09719],
                    [-5.78783, 1.39557, 2.6702, 0.10434],
                    [-6.06306, 1.38631, 2.6533, 0.10379],
                    [-6.33368, 1.36342, 2.55155, 0.10644],
                    [-6.59842, 1.32384, 2.45161, 0.10919],
                    [-6.85582, 1.2646, 2.35942, 0.11195],
                    [-7.10422, 1.18286, 2.26738, 0.11533],
                    [-7.34145, 1.07564, 2.17867, 0.11949],
                    [-7.56644, 0.9431, 2.08654, 0.12515],
                    [-7.77679, 0.78321, 1.99897, 0.13217],
                    [-7.96895, 0.59333, 1.91229, 0.14127],
                    [-8.13746, 0.37051, 1.83045, 0.15262],
                    [-8.27288, 0.11425, 1.64671, 0.17601],
                    [-8.35942, -0.15588, 1.64671, 0.17225],
                    [-8.39277, -0.41799, 1.64671, 0.16046],
                    [-8.37799, -0.6644, 1.64671, 0.14991],
                    [-8.31973, -0.89105, 1.64671, 0.14211],
                    [-8.22123, -1.09406, 1.64671, 0.13703],
                    [-8.08015, -1.26426, 1.73349, 0.12753],
                    [-7.90579, -1.40068, 1.82778, 0.12113],
                    [-7.70467, -1.50256, 1.93349, 0.1166],
                    [-7.48201, -1.56909, 2.05707, 0.11297],
                    [-7.24241, -1.59971, 2.20278, 0.10966],
                    [-6.99018, -1.59437, 2.38156, 0.10593],
                    [-6.72941, -1.55394, 2.43141, 0.10853],
                    [-6.46377, -1.48016, 2.32918, 0.11836],
                    [-6.20936, -1.38185, 2.06386, 0.13216],
                    [-5.95603, -1.3124, 2.06386, 0.12728],
                    [-5.70444, -1.27167, 2.06386, 0.12349],
                    [-5.45544, -1.26004, 2.06386, 0.12078],
                    [-5.21019, -1.27856, 2.06386, 0.11917],
                    [-4.97032, -1.329, 2.06386, 0.11877],
                    [-4.73996, -1.4199, 2.39219, 0.10352],
                    [-4.51885, -1.54159, 2.94261, 0.08577],
                    [-4.30485, -1.68453, 4.0, 0.06434],
                    [-4.09473, -1.83854, 4.0, 0.06513],
                    [-3.86984, -1.9996, 4.0, 0.06915],
                    [-3.64193, -2.15751, 4.0, 0.06932],
                    [-3.41033, -2.31123, 4.0, 0.06949],
                    [-3.17445, -2.45962, 4.0, 0.06967],
                    [-2.93376, -2.60147, 4.0, 0.06985],
                    [-2.68777, -2.73542, 4.0, 0.07002],
                    [-2.43603, -2.85998, 4.0, 0.07022],
                    [-2.17808, -2.97352, 4.0, 0.07046],
                    [-1.9144, -3.07628, 4.0, 0.07075],
                    [-1.64537, -3.16845, 4.0, 0.0711],
                    [-1.37124, -3.25022, 4.0, 0.07152],
                    [-1.09227, -3.32172, 4.0, 0.072],
                    [-0.80869, -3.38304, 4.0, 0.07253],
                    [-0.52071, -3.43424, 4.0, 0.07312],
                    [-0.22859, -3.47533, 4.0, 0.07375],
                    [0.0674, -3.50624, 4.0, 0.0744],
                    [0.36666, -3.52682, 4.0, 0.07499],
                    [0.6666, -3.5367, 4.0, 0.07502],
                    [0.96461, -3.53583, 4.0, 0.0745],
                    [1.26022, -3.52425, 4.0, 0.07396],
                    [1.55319, -3.50193, 4.0, 0.07345],
                    [1.84329, -3.46875, 4.0, 0.073],
                    [2.13031, -3.4246, 4.0, 0.0726],
                    [2.41402, -3.36929, 4.0, 0.07226],
                    [2.69421, -3.30265, 4.0, 0.072],
                    [2.97061, -3.22444, 4.0, 0.07181],
                    [3.24294, -3.13441, 4.0, 0.07171],
                    [3.51097, -3.03245, 4.0, 0.07169],
                    [3.77446, -2.91857, 4.0, 0.07176],
                    [4.03326, -2.79294, 4.0, 0.07192],
                    [4.28723, -2.65584, 4.0, 0.07215],
                    [4.53624, -2.50763, 4.0, 0.07245],
                    [4.78028, -2.34884, 4.0, 0.07279],
                    [5.01937, -2.18012, 4.0, 0.07316],
                    [5.25356, -2.00216, 4.0, 0.07353],
                    [5.48298, -1.81575, 4.0, 0.0739],
                    [5.70776, -1.62161, 4.0, 0.07425],
                    [5.92809, -1.42057, 4.0, 0.07457],
                    [6.14419, -1.21343, 4.0, 0.07484],
                    [6.35629, -1.00097, 4.0, 0.07505],
                    [6.56465, -0.78394, 4.0, 0.07521],
                    [6.76955, -0.56305, 4.0, 0.07532],
                    [6.97129, -0.33895, 3.3139, 0.09099],
                    [7.1702, -0.11221, 2.61597, 0.1153],
                    [7.36663, 0.1167, 2.22715, 0.13544],
                    [7.561, 0.34737, 1.96934, 0.15317],
                    [7.75333, 0.57975, 1.77423, 0.17001],
                    [7.93592, 0.81518, 1.6223, 0.18366],
                    [8.09318, 1.05404, 1.5, 0.19065],
                    [8.21346, 1.29429, 1.5, 0.17912],
                    [8.29021, 1.53248, 1.5, 0.16684],
                    [8.32048, 1.76472, 1.5, 0.15613],
                    [8.30223, 1.98661, 1.5, 0.14843],
                    [8.23261, 2.19251, 1.5, 0.1449],
                    [8.1062, 2.37324, 1.75876, 0.1254],
                    [7.93782, 2.52951, 1.92174, 0.11954],
                    [7.73164, 2.65915, 2.13983, 0.11382],
                    [7.49223, 2.76065, 2.42855, 0.10708],
                    [7.22679, 2.83376, 2.87192, 0.09587],
                    [6.95575, 2.88072, 3.68076, 0.07474]]

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