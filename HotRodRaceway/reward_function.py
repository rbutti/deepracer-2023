import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

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
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            try:
                distance = abs(-(a ** 4) + 2 * (a ** 2) * (b ** 2) + 2 * (a ** 2) * (c ** 2) -
                               (b ** 4) + 2 * (b ** 2) * (c ** 2) - (c ** 4)) ** 0.5 / (2 * a)
            except:
                distance = b

            return distance

        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0] + heading_vector[0],
                              car_coords[1] + heading_vector[1]]

            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

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

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count - 1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time / current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

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
                        [4.23813, 3.18241, 3.73945, 0.08066],
                        [3.93831, 3.21555, 3.36457, 0.08965],
                        [3.63849, 3.24867, 3.07867, 0.09798],
                        [3.33867, 3.28177, 2.8483, 0.1059],
                        [3.03912, 3.31394, 2.65258, 0.11358],
                        [2.74041, 3.34329, 2.48133, 0.12096],
                        [2.44312, 3.36795, 2.33406, 0.12781],
                        [2.14784, 3.38615, 2.19987, 0.13448],
                        [1.85515, 3.3962, 1.81638, 0.16123],
                        [1.56566, 3.3965, 1.81638, 0.15938],
                        [1.27997, 3.38557, 1.81638, 0.1574],
                        [0.9987, 3.36201, 1.81638, 0.1554],
                        [0.7225, 3.32445, 1.81638, 0.15346],
                        [0.4521, 3.27148, 1.81638, 0.1517],
                        [0.1883, 3.20171, 1.81638, 0.15023],
                        [-0.06793, 3.11358, 1.81638, 0.14918],
                        [-0.31258, 2.99916, 2.10567, 0.12826],
                        [-0.54821, 2.86536, 2.58807, 0.1047],
                        [-0.77772, 2.7186, 2.56266, 0.1063],
                        [-1.00404, 2.56512, 2.38316, 0.11475],
                        [-1.23658, 2.41084, 2.2694, 0.12297],
                        [-1.47176, 2.26061, 2.2694, 0.12297],
                        [-1.71041, 2.11629, 2.2694, 0.12289],
                        [-1.95317, 1.97983, 2.2694, 0.12271],
                        [-2.20057, 1.8532, 2.2694, 0.12247],
                        [-2.45305, 1.73845, 2.2694, 0.12221],
                        [-2.71104, 1.6378, 2.2694, 0.12203],
                        [-2.97494, 1.5536, 2.2694, 0.12206],
                        [-3.24483, 1.4878, 2.41529, 0.11502],
                        [-3.5196, 1.43856, 2.58802, 0.10786],
                        [-3.79834, 1.40403, 2.79527, 0.10048],
                        [-4.0803, 1.3824, 2.64013, 0.10711],
                        [-4.36484, 1.3718, 2.33518, 0.12193],
                        [-4.65138, 1.37035, 2.10328, 0.13624],
                        [-4.93943, 1.37605, 1.9263, 0.14956],
                        [-5.2285, 1.38685, 1.78013, 0.1625],
                        [-5.50923, 1.39453, 1.76887, 0.15877],
                        [-5.78783, 1.39557, 1.70104, 0.16378],
                        [-6.06306, 1.38631, 1.63441, 0.16849],
                        [-6.33368, 1.36342, 1.57294, 0.17266],
                        [-6.59842, 1.32384, 1.51159, 0.17709],
                        [-6.85582, 1.2646, 1.45245, 0.18185],
                        [-7.10422, 1.18286, 1.39103, 0.18799],
                        [-7.34145, 1.07564, 1.33265, 0.19535],
                        [-7.56644, 0.9431, 1.27486, 0.20483],
                        [-7.77679, 0.78321, 1.2203, 0.21652],
                        [-7.96895, 0.59333, 1.09781, 0.24608],
                        [-8.13746, 0.37051, 1.09781, 0.25448],
                        [-8.27288, 0.11425, 1.09781, 0.26402],
                        [-8.35942, -0.15588, 1.09781, 0.25838],
                        [-8.39277, -0.41799, 1.09781, 0.24069],
                        [-8.37799, -0.6644, 1.09781, 0.22486],
                        [-8.31973, -0.89105, 1.09781, 0.21316],
                        [-8.22123, -1.09406, 1.09781, 0.20555],
                        [-8.08015, -1.26426, 1.15566, 0.19129],
                        [-7.90579, -1.40068, 1.21852, 0.18169],
                        [-7.70467, -1.50256, 1.28899, 0.1749],
                        [-7.48201, -1.56909, 1.37138, 0.16946],
                        [-7.24241, -1.59971, 1.46852, 0.16448],
                        [-6.99018, -1.59437, 1.55279, 0.16247],
                        [-6.72941, -1.55394, 1.37591, 0.19179],
                        [-6.46377, -1.48016, 1.37591, 0.20037],
                        [-6.20936, -1.38185, 1.37591, 0.19823],
                        [-5.95603, -1.3124, 1.37591, 0.19091],
                        [-5.70444, -1.27167, 1.37591, 0.18524],
                        [-5.45544, -1.26004, 1.37591, 0.18117],
                        [-5.21019, -1.27856, 1.37591, 0.17875],
                        [-4.97032, -1.329, 1.37591, 0.17815],
                        [-4.73996, -1.4199, 1.5948, 0.15528],
                        [-4.51885, -1.54159, 1.96174, 0.12866],
                        [-4.30485, -1.68453, 2.77143, 0.09286],
                        [-4.09473, -1.83854, 2.8489, 0.09145],
                        [-3.86984, -1.9996, 2.8489, 0.09709],
                        [-3.64193, -2.15751, 2.8489, 0.09733],
                        [-3.41033, -2.31123, 2.8489, 0.09757],
                        [-3.17445, -2.45962, 2.8489, 0.09782],
                        [-2.93376, -2.60147, 2.8489, 0.09807],
                        [-2.68777, -2.73542, 2.8489, 0.09832],
                        [-2.43603, -2.85998, 2.8489, 0.09859],
                        [-2.17808, -2.97352, 2.91448, 0.0967],
                        [-1.9144, -3.07628, 2.97358, 0.09517],
                        [-1.64537, -3.16845, 3.03027, 0.09385],
                        [-1.37124, -3.25022, 3.08613, 0.09269],
                        [-1.09227, -3.32172, 3.14148, 0.09167],
                        [-0.80869, -3.38304, 3.19134, 0.09091],
                        [-0.52071, -3.43424, 3.16419, 0.09244],
                        [-0.22859, -3.47533, 3.11837, 0.0946],
                        [0.0674, -3.50624, 3.06706, 0.09703],
                        [0.36666, -3.52682, 3.01708, 0.09942],
                        [0.6666, -3.5367, 2.96843, 0.1011],
                        [0.96461, -3.53583, 2.92185, 0.102],
                        [1.26022, -3.52425, 2.89709, 0.10211],
                        [1.55319, -3.50193, 2.89093, 0.10163],
                        [1.84329, -3.46875, 2.89093, 0.101],
                        [2.13031, -3.4246, 2.89093, 0.10045],
                        [2.41402, -3.36929, 2.89093, 0.09999],
                        [2.69421, -3.30265, 2.89093, 0.09962],
                        [2.97061, -3.22444, 2.89093, 0.09936],
                        [3.24294, -3.13441, 2.89093, 0.09922],
                        [3.51097, -3.03245, 2.89093, 0.09919],
                        [3.77446, -2.91857, 2.90692, 0.09875],
                        [4.03326, -2.79294, 2.94263, 0.09776],
                        [4.28723, -2.65584, 2.9902, 0.09652],
                        [4.53624, -2.50763, 3.06593, 0.09452],
                        [4.78028, -2.34884, 3.16664, 0.09194],
                        [5.01937, -2.18012, 3.28079, 0.08919],
                        [5.25356, -2.00216, 3.42457, 0.08589],
                        [5.48298, -1.81575, 3.57123, 0.08278],
                        [5.70776, -1.62161, 3.76052, 0.07898],
                        [5.92809, -1.42057, 3.96437, 0.07524],
                        [6.14419, -1.21343, 4.0, 0.07484],
                        [6.35629, -1.00097, 3.49695, 0.08585],
                        [6.56465, -0.78394, 2.20927, 0.13618],
                        [6.76955, -0.56305, 1.74398, 0.17276],
                        [6.97129, -0.33895, 1.48476, 0.20308],
                        [7.1702, -0.11221, 1.3129, 0.22974],
                        [7.36663, 0.1167, 1.18282, 0.25502],
                        [7.561, 0.34737, 1.08153, 0.2789],
                        [7.75333, 0.57975, 1.0, 0.30164],
                        [7.93592, 0.81518, 1.0, 0.29794],
                        [8.09318, 1.05404, 1.0, 0.28598],
                        [8.21346, 1.29429, 1.0, 0.26867],
                        [8.29021, 1.53248, 1.0, 0.25026],
                        [8.32048, 1.76472, 1.0, 0.2342],
                        [8.30223, 1.98661, 1.0, 0.22264],
                        [8.23261, 2.19251, 1.0, 0.21735],
                        [8.1062, 2.37324, 1.1725, 0.1881],
                        [7.93782, 2.52951, 1.28116, 0.17931],
                        [7.73164, 2.65915, 1.42655, 0.17073],
                        [7.49223, 2.76065, 1.61904, 0.16061],
                        [7.22679, 2.83376, 1.91461, 0.1438],
                        [6.95575, 2.88072, 2.45384, 0.1121]]

        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist / (track_width * 0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2] - speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            speed_reward = (1 - (speed_diff / (SPEED_DIFF_NO_REWARD)) ** 2) ** 2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME * (FASTEST_TIME) /
                                           (STANDARD_TIME - FASTEST_TIME)) * (
                                            steps_prediction - (STANDARD_TIME * 15 + 1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        speed_diff_zero = optimals[2] - speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        REWARD_FOR_FASTEST_TIME = 1500  # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15 * (STANDARD_TIME - FASTEST_TIME))) * (steps - STANDARD_TIME * 15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        return float(reward)


reward_object = Reward()


def reward_function(params):
    return reward_object.reward_function(params)
