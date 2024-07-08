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

        racing_track = [[3.45842, 1.32401, 2.77018, 0.0771],
                        [3.24932, 1.37884, 2.99764, 0.07211],
                        [3.03276, 1.42179, 3.27172, 0.06748],
                        [2.80717, 1.45439, 3.61968, 0.06297],
                        [2.56987, 1.47799, 4.0, 0.05962],
                        [2.31707, 1.49377, 4.0, 0.06332],
                        [2.0453, 1.5029, 4.0, 0.06798],
                        [1.75965, 1.50679, 4.0, 0.07142],
                        [1.47001, 1.50789, 4.0, 0.07241],
                        [1.17828, 1.50709, 4.0, 0.07293],
                        [0.88496, 1.50484, 4.0, 0.07333],
                        [0.59053, 1.50161, 4.0, 0.07361],
                        [0.29552, 1.4979, 4.0, 0.07376],
                        [-0.00604, 1.49442, 4.0, 0.07539],
                        [-0.30777, 1.49124, 4.0, 0.07544],
                        [-0.60967, 1.48842, 4.0, 0.07548],
                        [-0.91179, 1.48611, 4.0, 0.07553],
                        [-1.21412, 1.48446, 4.0, 0.07558],
                        [-1.51666, 1.48363, 3.65737, 0.08272],
                        [-1.81934, 1.48377, 2.79659, 0.10823],
                        [-2.12209, 1.48507, 2.3393, 0.12942],
                        [-2.42482, 1.48767, 2.0488, 0.14776],
                        [-2.72745, 1.49178, 1.83862, 0.16461],
                        [-3.01756, 1.49105, 1.59795, 0.18155],
                        [-3.29311, 1.47579, 1.59795, 0.17271],
                        [-3.55086, 1.43925, 1.59795, 0.16291],
                        [-3.78831, 1.37706, 1.59795, 0.15361],
                        [-4.00251, 1.28695, 1.59795, 0.14542],
                        [-4.18939, 1.16827, 1.59795, 0.13854],
                        [-4.34005, 1.01997, 1.61955, 0.13053],
                        [-4.45485, 0.8504, 1.63347, 0.12536],
                        [-4.53404, 0.66581, 1.64643, 0.122],
                        [-4.57773, 0.47127, 1.65869, 0.12021],
                        [-4.58564, 0.27126, 1.66952, 0.1199],
                        [-4.55706, 0.07009, 1.671, 0.12159],
                        [-4.49048, -0.12755, 1.671, 0.12481],
                        [-4.38381, -0.316, 1.80827, 0.11975],
                        [-4.241, -0.49116, 1.92538, 0.11738],
                        [-4.06427, -0.64955, 2.06364, 0.115],
                        [-3.85589, -0.78842, 2.23061, 0.11226],
                        [-3.61829, -0.90573, 2.44183, 0.10852],
                        [-3.35461, -1.00034, 2.7241, 0.10284],
                        [-3.07038, -1.07236, 3.11258, 0.0942],
                        [-2.77607, -1.12347, 3.67381, 0.08131],
                        [-2.47963, -1.15824, 4.0, 0.07462],
                        [-2.18324, -1.18037, 4.0, 0.0743],
                        [-1.88865, -1.19192, 4.0, 0.0737],
                        [-1.59737, -1.19491, 4.0, 0.07282],
                        [-1.30938, -1.19133, 4.0, 0.072],
                        [-1.02398, -1.18298, 4.0, 0.07138],
                        [-0.74033, -1.17152, 4.0, 0.07097],
                        [-0.45753, -1.15855, 4.0, 0.07077],
                        [-0.1855, -1.14648, 4.0, 0.06807],
                        [0.08583, -1.1354, 4.0, 0.06789],
                        [0.35615, -1.12587, 4.0, 0.06762],
                        [0.6253, -1.11842, 4.0, 0.06731],
                        [0.8932, -1.1136, 4.0, 0.06699],
                        [1.15987, -1.1119, 4.0, 0.06667],
                        [1.42534, -1.11388, 4.0, 0.06637],
                        [1.68968, -1.12004, 4.0, 0.0661],
                        [1.95292, -1.13097, 3.77987, 0.0697],
                        [2.21507, -1.14726, 2.77144, 0.09477],
                        [2.47614, -1.16955, 2.29012, 0.11441],
                        [2.73613, -1.19852, 1.99388, 0.1312],
                        [2.99505, -1.23495, 1.78617, 0.14638],
                        [3.23261, -1.27587, 1.62896, 0.14799],
                        [3.46722, -1.30628, 1.5, 0.15771],
                        [3.69638, -1.31849, 1.5, 0.15299],
                        [3.91765, -1.30651, 1.5, 0.14772],
                        [4.12832, -1.26578, 1.5, 0.14305],
                        [4.32504, -1.19238, 1.5, 0.13998],
                        [4.50294, -1.08223, 1.5, 0.13949],
                        [4.65372, -0.93019, 1.5557, 0.13764],
                        [4.77189, -0.73952, 1.61764, 0.13867],
                        [4.85012, -0.51386, 1.68223, 0.14198],
                        [4.87954, -0.26268, 1.74816, 0.14466],
                        [4.85583, -0.00602, 1.81721, 0.14184],
                        [4.78502, 0.23576, 1.88923, 0.13335],
                        [4.678, 0.45302, 1.96533, 0.12323],
                        [4.54437, 0.64371, 2.04508, 0.11386],
                        [4.39117, 0.80885, 2.12946, 0.10578],
                        [4.2233, 0.95039, 2.23163, 0.09839],
                        [4.04423, 1.07066, 2.3352, 0.09237],
                        [3.85623, 1.17173, 2.45462, 0.08696],
                        [3.66073, 1.25556, 2.59782, 0.08189]]

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
