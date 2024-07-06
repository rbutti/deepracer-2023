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

        racing_track = [[2.08696, 0.99616, 0.54925, 0.17742],
                        [2.18493, 0.9982, 0.56927, 0.17213],
                        [2.28309, 1.00873, 0.59361, 0.16631],
                        [2.38098, 1.0272, 0.62681, 0.15894],
                        [2.47826, 1.05294, 0.66589, 0.15112],
                        [2.57469, 1.08525, 0.70789, 0.14365],
                        [2.67009, 1.12347, 0.71766, 0.1432],
                        [2.76437, 1.16695, 0.70937, 0.14636],
                        [2.85749, 1.21503, 0.70695, 0.14825],
                        [2.94947, 1.26709, 0.70403, 0.15012],
                        [3.04317, 1.31619, 0.70403, 0.15025],
                        [3.13792, 1.36154, 0.70403, 0.1492],
                        [3.23372, 1.40273, 0.70403, 0.14812],
                        [3.33054, 1.43937, 0.70403, 0.14704],
                        [3.42832, 1.47109, 0.70403, 0.14602],
                        [3.527, 1.49759, 0.70403, 0.14512],
                        [3.62645, 1.51863, 0.70403, 0.14438],
                        [3.72654, 1.53403, 0.70403, 0.14385],
                        [3.82713, 1.54369, 0.70403, 0.14353],
                        [3.92805, 1.54758, 0.70403, 0.14345],
                        [4.02914, 1.54564, 0.70721, 0.14297],
                        [4.13023, 1.53789, 0.71404, 0.14198],
                        [4.23113, 1.52439, 0.72457, 0.1405],
                        [4.33168, 1.50526, 0.70062, 0.14609],
                        [4.4317, 1.48065, 0.65429, 0.15743],
                        [4.53104, 1.45076, 0.61735, 0.16804],
                        [4.62955, 1.41586, 0.58576, 0.17842],
                        [4.72709, 1.37618, 0.55593, 0.18943],
                        [4.82353, 1.33194, 0.53681, 0.19765],
                        [4.91872, 1.28336, 0.52067, 0.20525],
                        [5.0125, 1.23067, 0.50977, 0.21102],
                        [5.10076, 1.17655, 0.50377, 0.20551],
                        [5.18921, 1.12752, 0.5, 0.20227],
                        [5.2775, 1.08422, 0.5, 0.19667],
                        [5.36537, 1.04706, 0.5, 0.1908],
                        [5.45262, 1.01639, 0.5, 0.18497],
                        [5.53917, 0.99242, 0.5, 0.17962],
                        [5.62499, 0.97529, 0.5, 0.17502],
                        [5.71007, 0.96523, 0.5, 0.17134],
                        [5.79443, 0.96235, 0.5, 0.16882],
                        [5.87805, 0.96684, 0.5, 0.16749],
                        [5.96087, 0.97893, 0.5, 0.16738],
                        [6.04269, 0.99884, 0.5, 0.16842],
                        [6.12322, 1.02684, 0.50308, 0.16947],
                        [6.20201, 1.06307, 0.50541, 0.17158],
                        [6.27847, 1.10767, 0.51255, 0.1727],
                        [6.35188, 1.16058, 0.52101, 0.1737],
                        [6.42146, 1.22161, 0.53233, 0.17386],
                        [6.48642, 1.29034, 0.5459, 0.17322],
                        [6.54605, 1.36615, 0.56117, 0.17188],
                        [6.59977, 1.44835, 0.57946, 0.16947],
                        [6.64722, 1.53615, 0.60225, 0.16571],
                        [6.68822, 1.62873, 0.63109, 0.16044],
                        [6.72286, 1.72529, 0.66666, 0.15387],
                        [6.75142, 1.82507, 0.70991, 0.1462],
                        [6.77431, 1.92743, 0.76101, 0.13783],
                        [6.79207, 2.0318, 0.82011, 0.12909],
                        [6.80525, 2.1377, 0.78594, 0.13579],
                        [6.81448, 2.24476, 0.72636, 0.14793],
                        [6.8204, 2.35262, 0.67754, 0.15944],
                        [6.8237, 2.46102, 0.63667, 0.17034],
                        [6.82507, 2.56974, 0.60207, 0.18058],
                        [6.82519, 2.67859, 0.57402, 0.18963],
                        [6.82427, 2.78736, 0.55279, 0.19678],
                        [6.82146, 2.8959, 0.53645, 0.2024],
                        [6.81591, 3.00399, 0.52386, 0.2066],
                        [6.80681, 3.11135, 0.51444, 0.20943],
                        [6.79338, 3.21764, 0.5081, 0.21086],
                        [6.77489, 3.32247, 0.50506, 0.21077],
                        [6.75063, 3.42538, 0.50506, 0.20933],
                        [6.71994, 3.5258, 0.50506, 0.2079],
                        [6.68225, 3.62307, 0.50506, 0.20654],
                        [6.63707, 3.71638, 0.50506, 0.20527],
                        [6.58409, 3.80482, 0.50506, 0.20412],
                        [6.52325, 3.88737, 0.50506, 0.20305],
                        [6.45477, 3.96301, 0.50506, 0.20203],
                        [6.37912, 4.03072, 0.50506, 0.20101],
                        [6.29699, 4.08954, 0.50506, 0.20001],
                        [6.2093, 4.13871, 0.50506, 0.19907],
                        [6.11705, 4.17767, 0.50565, 0.19804],
                        [6.02132, 4.2061, 0.51016, 0.19575],
                        [5.92319, 4.22398, 0.51925, 0.1921],
                        [5.82365, 4.23149, 0.53163, 0.18776],
                        [5.72358, 4.229, 0.5456, 0.18346],
                        [5.62374, 4.21692, 0.56307, 0.17861],
                        [5.52475, 4.19578, 0.58518, 0.17297],
                        [5.42712, 4.16616, 0.61269, 0.16652],
                        [5.33121, 4.12876, 0.64337, 0.16001],
                        [5.23728, 4.08424, 0.6629, 0.15681],
                        [5.14551, 4.03323, 0.66195, 0.15861],
                        [5.056, 3.97634, 0.66195, 0.16022],
                        [4.96882, 3.91411, 0.66195, 0.16181],
                        [4.87828, 3.85591, 0.66195, 0.1626],
                        [4.78538, 3.80279, 0.66195, 0.16167],
                        [4.69026, 3.75517, 0.66195, 0.1607],
                        [4.5931, 3.71345, 0.66195, 0.15973],
                        [4.49415, 3.67798, 0.66195, 0.15879],
                        [4.39369, 3.64911, 0.66195, 0.15792],
                        [4.29203, 3.62702, 0.66195, 0.15716],
                        [4.18952, 3.61184, 0.66195, 0.15655],
                        [4.08651, 3.60357, 0.6636, 0.15574],
                        [3.98331, 3.60215, 0.67276, 0.1534],
                        [3.88026, 3.60739, 0.68196, 0.15132],
                        [3.7776, 3.61911, 0.69804, 0.14801],
                        [3.6756, 3.63702, 0.71842, 0.14416],
                        [3.57444, 3.66078, 0.71997, 0.14433],
                        [3.47427, 3.69003, 0.66352, 0.15727],
                        [3.37522, 3.72441, 0.61832, 0.16957],
                        [3.27735, 3.76348, 0.58254, 0.18089],
                        [3.1807, 3.80678, 0.55711, 0.19011],
                        [3.08528, 3.85396, 0.53814, 0.19781],
                        [2.99107, 3.90462, 0.52444, 0.20395],
                        [2.89808, 3.9584, 0.51572, 0.20831],
                        [2.80485, 4.00886, 0.5102, 0.20778],
                        [2.71058, 4.05542, 0.50841, 0.20679],
                        [2.6152, 4.09709, 0.50841, 0.20473],
                        [2.51873, 4.1329, 0.50841, 0.20241],
                        [2.42129, 4.16193, 0.50841, 0.19998],
                        [2.32316, 4.18333, 0.50841, 0.19756],
                        [2.22473, 4.19634, 0.50841, 0.19528],
                        [2.12654, 4.20039, 0.50841, 0.1933],
                        [2.02923, 4.19508, 0.50841, 0.19169],
                        [1.93352, 4.18017, 0.50841, 0.19052],
                        [1.84019, 4.15559, 0.50841, 0.18983],
                        [1.75006, 4.12141, 0.50841, 0.18961],
                        [1.66392, 4.07786, 0.51024, 0.18917],
                        [1.58256, 4.02531, 0.51567, 0.18783],
                        [1.50664, 3.96432, 0.52454, 0.18565],
                        [1.43675, 3.89553, 0.53667, 0.18273],
                        [1.37331, 3.81969, 0.55215, 0.17907],
                        [1.31662, 3.73757, 0.57071, 0.17485],
                        [1.26681, 3.64997, 0.59166, 0.17031],
                        [1.2239, 3.55766, 0.61834, 0.16463],
                        [1.18774, 3.46137, 0.65154, 0.15786],
                        [1.158, 3.36183, 0.69448, 0.1496],
                        [1.13418, 3.25968, 0.74444, 0.14089],
                        [1.11574, 3.1555, 0.80217, 0.1319],
                        [1.10207, 3.04975, 0.75987, 0.14033],
                        [1.09254, 2.94283, 0.70187, 0.15294],
                        [1.08646, 2.83507, 0.65494, 0.16479],
                        [1.08313, 2.72676, 0.61627, 0.17585],
                        [1.08185, 2.61809, 0.5845, 0.18593],
                        [1.08192, 2.5092, 0.5603, 0.19433],
                        [1.08299, 2.40034, 0.54146, 0.20107],
                        [1.08602, 2.29174, 0.52695, 0.20617],
                        [1.09191, 2.18366, 0.51625, 0.20967],
                        [1.10155, 2.07642, 0.50912, 0.21148],
                        [1.11577, 1.97043, 0.50587, 0.2114],
                        [1.13535, 1.86617, 0.50587, 0.20972],
                        [1.16099, 1.76418, 0.50587, 0.20788],
                        [1.19331, 1.66513, 0.50587, 0.20596],
                        [1.23281, 1.56978, 0.50587, 0.20402],
                        [1.27983, 1.479, 0.50587, 0.20211],
                        [1.33448, 1.39369, 0.50587, 0.20027],
                        [1.39662, 1.3148, 0.50587, 0.19852],
                        [1.46589, 1.24325, 0.50587, 0.19686],
                        [1.5417, 1.17989, 0.50587, 0.19532],
                        [1.62332, 1.12541, 0.50587, 0.19398],
                        [1.70986, 1.08033, 0.50697, 0.19247],
                        [1.80039, 1.04492, 0.51233, 0.18974],
                        [1.89397, 1.01921, 0.52159, 0.18607],
                        [1.98976, 1.00306, 0.53549, 0.18139]]

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
