import math
import json
import os



class HyperParams:
    NOT_WHEELS_TRACK_PENALTY=0.001

    MAX_SPEED = 6
    MIN_SPEED = 0
    USE_SPEED_PENALTY = False
    SPEED_START_PENALITY = 0.5

    FUTURE_WAYPOINT_NUMBER = 20

    USE_CURVATURE = False
    CURVATURE_WAYPOINT_NUMBER = 4
    MAX_TRACK_DIFF = 4

    USE_POSITION = False
    POSITION_WAYPOINT_NUMBER = 10
    MIN_THETA=20

    USE_STEER = False
    STEERING_WAYPOINT_NUMBER = 5
    STEERING_THRESHOLD = 10.0
    MIN_STEER_THETA = 0.04
    STEERING_PENALTY = 0.2

    USE_DISPLACEMENT = False
    DIRECTION_THRESHOLD = 10.0
    DIRECTION_PENALTY = 0.55

    USE_BORDER_DISTANCE = False
    USE_CENTER_DISTANCE = True
    MIN_DIST_FROM_BORDER = 0.2
    CENTER_IMPORTANCE=0.5
    BORDER_IMPORTANCE=0.6

    CAR_WIDTH = 0.06

    def __init__(self):
        print("Hyper params initialized")


hp = HyperParams()


class Point:

    def __init__(self, coord):
        assert len(coord) == 2
        self.x = coord[0]
        self.y = coord[1]

    def difference(self, point):
        """ Return the difference between two points"""
        return (abs(self.x - point.x), abs(self.y - point.y))

    def direction_degree(self, prev_point):
        # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
        track_direction = math.atan2(self.y - prev_point.y, self.x - prev_point.y)
        # Convert to degree
        track_direction = math.degrees(track_direction)
        return track_direction

    def line_passing(self, pointb):
        """Get the coefficents of the line passing between two points"""

        try:
            m = (pointb.y - self.y) / (pointb.x - self.x)
        except ZeroDivisionError:
            m = 0

        c = self.y - m * self.x

        return (m, c)

    def get_anlge(self, pointb):
        m1 = (self.y - self.y) / 1
        m2 = (pointb.y - self.y) / (pointb.x - self.x)

        tnAngle = (m1 - m2) / (1 + (m1 * m2))
        return math.atan(tnAngle)

def angle_between_lines(m1, m2):
    """Return the angle between two lines"""

    angle = (m1 - m2) / (1 + (m1 * m2))
    return math.degrees(math.atan(angle))

def get_total_angle_car(car_pos, next_wps):
    """
    car_pos: class Point, represent the postion of the car
    heading: float, the angle of heading
    next_wps: list of Point, next waypoints

    return: angle between car heading and waypoints

    """



    theta = 0

    for idx in range(len(next_wps) - 1):
        p1 = next_wps[idx]
        p2 = next_wps[idx + 1]

        # plt1 = plt.scatter(p1.x, p1.y, c="blue", alpha=0.7)
        # plt2 = plt.scatter(p2.x, p2.y, c="blue", alpha=0.7)

        # get thte slope of the line passing trought the car postiion and the next waypoint
        slope1 = car_pos.line_passing(p1)[0]
        slope2 = car_pos.line_passing(p2)[0]

        # if idx==0:
        #    slope1=car_slope

        # add the angle
        prov = angle_between_lines(slope1, slope2)
        theta += prov

        # plt1.remove()
        # plt2.remove()

    return theta


def get_total_angle_absolute(next_wps):
    """
    car_pos: class Point, represent the postion of the car
    heading: float, the angle of heading
    next_wps: list of Point, next waypoints

    return: angle between car heading and waypoints

    """



    theta = 0

    for idx in range(len(next_wps) - 2):
        p1 = next_wps[idx]
        p2 = next_wps[idx + 1]
        p3 = next_wps[idx + 2]

        # plt1 = plt.scatter(p1.x, p1.y, c="blue", alpha=0.7)
        # plt2 = plt.scatter(p2.x, p2.y, c="blue", alpha=0.7)
        # plt3 = plt.scatter(p3.x, p3.y, c="blue", alpha=0.7)

        # get thte slope of the line passing trought the car postiion and the next waypoint
        slope1 = p1.line_passing(p2)[0]
        slope2 = p2.line_passing(p3)[0]

        # add the angle
        prov = angle_between_lines(slope1, slope2)
        theta += abs(prov)

        # plt1.remove()
        # plt2.remove()
        # plt3.remove()

    return theta


def position_reward(theta, distance_from_center, track_width, is_left, reward):
    """ The car should be on the opposite side of an incoming curve, 
    e.g. curve is to the right, then the car should move to the left lane to make the curve easier"""

    # imagine to split the line into 4 equal pieces,
    # if i have a curve on the right i want to move the car to the left
    # so that the car center is aligned with the line splitting the left lane in two,
    #  same thing goes for the right curve

    tw4=track_width/4
    tw2=track_width/2

    # suppose a curve to the right, if the car is in the right lane but too right
    #  (right of the line splitting the lane) then the distance from the new center lane is the dc-tw/4
    if distance_from_center > tw4:
        new_center_distance = distance_from_center - tw4
    else: #reverse thing here
        new_center_distance = tw4- distance_from_center

    # curve to the right, must go to the left
    # car is on the right lane (wrong),we add the lenght of the right lane to the distance
    if theta > 0 and not is_left:
        # print("curve on right\n")
        new_center_distance += tw2

    # curve on the left, must go to the right
    # same thing for the other direction
    elif theta <= 0 and is_left:
        new_center_distance += tw2

    # the farthest distance from the correct center lane is all the way to the opposite side
    worst_distance=tw4+tw2

    multiplier =1- new_center_distance/worst_distance# the higher the closer to the correct center

    # print("curve penalty: {:f}\n".format(distance_center_perc))
    reward *= multiplier

    return reward


def curvature_reward(theta, speed, reward):
    """Penalize reward for speed/curvature relationship"""

    theta = abs(theta)
    print(theta)

    # get the curvature in percentage
    theta_perc = theta / hp.MAX_TRACK_DIFF

    speed_perc = speed / hp.MAX_SPEED


    # the higer the speed the lower the angle must be, in order to make smooth curve

    multiplier = math.pow(abs(theta_perc-speed_perc),1/2)
    multiplier = max(multiplier, 0.01)
    multiplier = min(multiplier, 1)

    # print("curvature penalty: {:f}\n".format(multiplier))
    return reward * multiplier


def heading_displacement_reward(nxt_wp, prv_wp, heading, reward):
    """
    Penalize based on heading displacement
    :param nxt_wp: next waypoint
    :param prv_wp: previous waypoint
    :param heading: heading
    :param reward: reward
    :return: reward
    """

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(nxt_wp[1] - prv_wp[1], nxt_wp[0] - prv_wp[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)

    # Penalize the reward if the difference is too large
    if direction_diff > hp.DIRECTION_THRESHOLD:
        reward *= hp.DIRECTION_PENALTY

    return reward


def track_distance_reward(distance_from_center, track_width, reward):
    """
    Function to scale the reward based on the distance from the border
    """

    # get half the distance
    track_width = track_width / 2

    if hp.USE_BORDER_DISTANCE:

        # get the min allowed distance (in meters) from the border
        min_distance = track_width * hp.MIN_DIST_FROM_BORDER*hp.BORDER_IMPORTANCE

        # get the current distance from the border
        border_distance = abs(track_width - distance_from_center) - hp.CAR_WIDTH / 2

        # print("Border distance :{:f}".format(border_distance))

        # if the current distance is less than the minumum one penalize
        if border_distance < min_distance:
            # the smallest the distance to the border the higer the penalization effect
            multiplier = border_distance / min_distance
            multiplier = max(multiplier, 0.01)
            reward *= multiplier
            # print("Border distance penalty :{:f}".format(border_distance/min_distance))

    if hp.USE_CENTER_DISTANCE:
        multiplier = 1 - distance_from_center / (track_width/(hp.CENTER_IMPORTANCE) - hp.CAR_WIDTH / 2)
        multiplier = max(multiplier, 0.01)
        reward *= multiplier

    return reward


def append_record(record):
    with open('params.txt', 'a') as f:
        json.dump(record, f)
        f.write(os.linesep)


def steering_function(steering, theta, reward):
    # if track is sufficently linear
    if abs(theta) < hp.MIN_STEER_THETA:
        # Penalize if car steer too much to prevent zigzag
        if steering > hp.STEERING_THRESHOLD:
            reward *=  hp.STEERING_PENALTY

    return reward


def speed_reward(speed, reward):
    """Award higher speed and penilize slow speed"""

    # penalize slow speed, when start speed is zero
    if speed < hp.MIN_SPEED and speed > 0:
        speed_perc = speed / hp.MIN_SPEED + 0.001
        reward *= speed_perc
        print("Speed low: {:f}. Reward 2 :{:f}\n".format(speed, reward))

    increase = hp.SPEED_START_PENALITY / 2

    if speed <= 3:  # min speed
        speed_perc = hp.SPEED_START_PENALITY

    elif speed < hp.MAX_SPEED:
        speed_perc = hp.SPEED_START_PENALITY + increase

    else:
        speed_perc = hp.SPEED_START_PENALITY + increase * 2

    reward *= speed_perc

    return reward


def get_next_wp_inrange(waypoints, range_):
    """
    Function to get next waypoints in list
    :param waypoints: list of waypoints from params
    :param range_: tuple, range
    :return: list of Points
    """

    nxt_wps = []
    range_max = min(range_[1], len(waypoints))
    for idx in range(range_[0], range_max):
        nxt_wps.append(Point(waypoints[idx]))

    # if the waypoints start again from zero get the followings
    if range_[1] > len(waypoints):
        range_max = hp.FUTURE_WAYPOINT_NUMBER - (len(waypoints) - range_[0])
        for idx in range(range_max):
            nxt_wps.append(Point(waypoints[idx]))

    return nxt_wps


def get_prev_wp_inrange(waypoints, range_):
    """
    Function to get previous waypoints in list
    :param waypoints: list of waypoints from params
    :param range_: tuple, range
    :return: list of Points
    """

    prev_wps = []

    # if the waypoints start again from zero get the followings
    if range_[0] < 0:
        range_min = len(waypoints) - abs(range_[0])
        for idx in range(range_min, len(waypoints)):
            prev_wps.append(Point(waypoints[idx]))

    range_min = max(range_[0], 0)
    for idx in range(range_min, range_[1]):
        prev_wps.append(Point(waypoints[idx]))

    return prev_wps


def reward_function(params):
    '''
    Example of rewarding the agent to follow center line
    '''

    # Read input parameters
    wp_idx = params["closest_waypoints"]
    steering = params["steering_angle"]
    heading = params["heading"]
    speed = params["speed"]
    track_width = params["track_width"]
    distance_from_center = params["distance_from_center"]
    is_left_of_center = params["is_left_of_center"]
    waypoints = params["waypoints"]

    # list of next waypoints
    nxt_wps = get_next_wp_inrange(waypoints, (wp_idx[1], wp_idx[1] + hp.FUTURE_WAYPOINT_NUMBER))

    all_wheels_on_track = params['all_wheels_on_track']

    # initial reward
    reward = 10

    # print("Reward 0 :{:f}\n".format(reward))

    # penalize wheel not on track
    if not all_wheels_on_track:
        reward *=hp.NOT_WHEELS_TRACK_PENALTY
        print("Not wheel on track: Reward 1 :{:f}\n".format(reward))

    # penalize for border distance
    if (hp.USE_BORDER_DISTANCE or hp.USE_CENTER_DISTANCE):
        reward = track_distance_reward(distance_from_center, track_width, reward)

    if hp.USE_CURVATURE:
        # penalize for angle/speed
        theta_curvature = get_total_angle_absolute(nxt_wps[:hp.CURVATURE_WAYPOINT_NUMBER])

        # print("Theta curvature1 :{:f}\n".format(theta_curvature))
        reward = curvature_reward(theta_curvature, speed, reward)
        # print("Reward 4 :{:f}\n".format(reward))

    if hp.USE_POSITION:
        car_pos = Point((params['x'], params['y']))
        # penalize for angle/position
        theta_position = get_total_angle_car(car_pos, nxt_wps[2:hp.POSITION_WAYPOINT_NUMBER+2])

        if abs(theta_position)>=hp.MIN_THETA:
            # print("Theta position :{:f}\n".format(theta_position))
            reward = position_reward(theta_position, distance_from_center, track_width, is_left_of_center, reward)
            # print("Reward 5 :{:f}\n".format(reward))

    if hp.USE_STEER:  # penalize sterring/straight line

        # add previous point to the list since the car can come from a curve and has to steer
        # get previous 2 points
        prv = get_prev_wp_inrange(waypoints, (wp_idx[0] - 2, wp_idx[0]))
        point_list = prv + nxt_wps[:hp.STEERING_WAYPOINT_NUMBER]
        theta_steer = get_total_angle_absolute(point_list)
        reward = steering_function(steering, theta_steer, reward)
        print("theta_steer1 :{:f}\n".format(theta_steer))

    if hp.USE_DISPLACEMENT:
        # penalize per heading displacement
        nxt_wp = waypoints[wp_idx[1]]
        prv_wp = waypoints[wp_idx[0]]
        reward = heading_displacement_reward(nxt_wp, prv_wp, heading, reward)
        ##print("Reward 6 :{:f}\n".format(reward))

    if hp.USE_SPEED_PENALTY:
        reward = speed_reward(speed, reward)

    del params["waypoints"]
    # print(waypoints)
    # print(params)

    return float(reward)
