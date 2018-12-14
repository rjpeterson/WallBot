import math
import time
from rlbot.agents.base_agent import SimpleControllerState
from Util import *

class TurtleATBA:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        target_location = agent.ball
        target_speed = velocity_2d(agent.ball) + (distance_2d(agent.ball, agent.me) / 1.5)

        return turtle_controller(agent, target_location, target_speed)

class MoveToPost:
    """This class determines when the bot should enter a shadow-defensive state
    and will pass a the appropriate post coordinates to the Controller"""
    def __init__(self):
        self.expired = False

    def available(self, agent):
        return True

    def execute(self, agent):

        self.own_negative_post = Vector3([-700, 5100 * sign(agent.me.team), 20])
        self.own_negative_post_yaw = 1.5708
        self.own_positive_post = Vector3([700, 5100 * sign(agent.me.team), 20])
        self.own_positive_post_yaw = -1.5708

        # Determine which side of the field the ball is on.
        if agent.ball.location.data[0] < 0:
            target = self.own_negative_post
            target_orientation = self.own_negative_post_yaw
        else:
            target = self.own_positive_post
            target_orientation = self.own_positive_post_yaw

        speed = int(distance_2d(agent.me.location.data, target))
        # Move car to corresponding goalpost.
        # Face center of goal


        # Check location & orientation
        if close_enough(agent.me.location.data, target): ##and abs(agent.me.rotation.data[1] - target_orientation) < 0.1:
            speed = 0
            self.expired = True

        agent.renderer.begin_rendering()
        agent.renderer.draw_string_2d(1, 1, 1, 1, str(distance_2d(agent.me.location.data, target)), agent.renderer.white())
        agent.renderer.end_rendering()

        return frugal_controller(agent, target, speed)

def move_to_post_controller(agent, target, target_orientaion, speed):
    pass

class DefendShot:
    def __init__(self):
        self.expired = False
        self.own_negative_post = Vector3([-700, 5100 * sign(self.me.team), 20])
        self.own_positive_post = Vector3([700, 5100 * sign(self.me.team), 20])

    def available(self, agent):

        if close_enough(self.own_negative_post) or close_enough(self.own_positive_post):
            return True
        return False
    def execute(self):
        pass

class CollectBoost:
    def __init__(self):
        self.expired = False
    def available(self, agent):
        return True
    def execute(self):
        return
        ## Predict when and where ball will cross goalline
        ## Move to target location at target time
        ## Dodge & boost through ball toward nearest post
        ## If safe, grab more boost

class CalcShot:
    def __init__(self):
        self.expired = False

    def available(self, agent):
        if ball_ready(agent) and abs(agent.ball.location[1]) < 5050 and ball_project(agent) > 500 - (
                distance_2d(agent.ball, agent.me) / 2):
            return True
        return False

    def execute(self, agent):
        agent.controller = calc_controller

        # getting the coordinates of the goalposts
        left_post = Vector3([-sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
        right_post = Vector3([sign(agent.team) * 700, 5100 * -sign(agent.team), 200])
        center = Vector3([0, 5150 * -sign(agent.team), 200])

        # time stuff that we don't worry about yet
        time_guess = 0
        bloc = future(agent.ball, time_guess)

        # vectors from the goalposts to the ball & to Gosling
        ball_left = angle2(bloc, left_post)
        ball_right = angle2(bloc, right_post)
        agent_left = angle2(agent.me, left_post)
        agent_right = angle2(agent.me, right_post)

        # determining if we are left/right/inside of cone
        if agent_left > ball_left and agent_right > ball_right:
            goal_target = right_post
        elif agent_left > ball_left and agent_right < ball_right:
            goal_target = None
        elif agent_left < ball_left and agent_right < ball_right:
            goal_target = left_post
        else:
            goal_target = None

        if goal_target != None:
            # if we are outside the cone, this is the same as Gosling's old code
            goal_to_ball = (agent.ball.location - goal_target).normalize()
            goal_to_agent = (agent.me.location - goal_target).normalize()
            difference = goal_to_ball - goal_to_agent
            error = cap(abs(difference[0]) + abs(difference[1]), 1, 10)
        else:
            # if we are inside the cone, our line to follow is a vector from the ball to us (although it's still named 'goal_to_ball')
            goal_to_ball = (agent.me.location - agent.ball.location).normalize()
            error = cap(distance_2d(bloc, agent.me) / 1000, 0, 1)

        # this is measuring how fast the ball is traveling away from us if we were stationary
        ball_dpp_skew = cap(abs(dpp(agent.ball.location, agent.ball.velocity, agent.me.location, [0, 0, 0])) / 80, 1,
                            1.5)

        # same as Gosling's old distance calculation, but now we consider dpp_skew which helps us handle when the ball is moving
        target_distance = cap((40 + distance_2d(agent.ball.location, agent.me) * (error ** 2)) / 1.8, 0, 4000)
        target_location = agent.ball.location + Vector3(
            [(goal_to_ball[0] * target_distance) * ball_dpp_skew, goal_to_ball[1] * target_distance, 0])

        # this also adjusts the target location based on dpp
        ball_something = dpp(target_location, agent.ball.velocity, agent.me, [0, 0, 0]) ** 2

        if ball_something > 100:  # if we were stopped, and the ball is moving 100uu/s away from us
            ball_something = cap(ball_something, 0, 80)
            correction = agent.ball.velocity.normalize()
            correction = Vector3([correction[0] * ball_something, correction.data[1] * ball_something,
                                  correction.data[2] * ball_something])
            target_location += correction  # we're adding some component of the ball's velocity to the target position so that we are able to hit a faster moving ball better
            # it's important that this only happens when the ball is moving away from us.

        # another target adjustment that applies if the ball is close to the wall
        extra = 4120 - abs(target_location.data[0])
        if extra < 0:
            # we prevent our target from going outside the wall, and extend it so that Gosling gets closer to the wall before taking a shot, makes things more reliable
            target_location.data[0] = cap(target_location.data[0], -4120, 4120)
            target_location.data[1] = target_location.data[1] + (-sign(agent.team) * cap(extra, -500, 500))

        # getting speed, this would be a good place to modify because it's not very good
        target_local = return_local_location(agent.ball.location, agent.me)
        angle_to_target = cap(math.atan2(target_local.data[1], target_local.data[0]), -3, 3)
        distance_to_target = distance_2d(agent.me, target_location)
        speed = 2000 - (100 * (1 + angle_to_target) ** 2)

        # picking our rendered target color based on the speed we want to go
        colorRed = cap(int((speed / 2300) * 255), 0, 255)
        colorBlue = cap(255 - colorRed, 0, 255)

        # see the rendering tutorial on github about this, just drawing lines from the posts to the ball and one from the ball to the target
        agent.renderer.begin_rendering()
        agent.renderer.draw_line_3d(bloc.data, left_post.data, agent.renderer.create_color(255, 255, 0, 0))
        agent.renderer.draw_line_3d(bloc.data, right_post.data, agent.renderer.create_color(255, 0, 255, 0))

        agent.renderer.draw_line_3d(agent.ball.location.data, target_location.data,
                                    agent.renderer.create_color(255, colorRed, 0, colorBlue))
        agent.renderer.draw_rect_3d(target_location.data, 10, 10, True,
                                    agent.renderer.create_color(255, colorRed, 0, colorBlue))
        agent.renderer.end_rendering()

        if ball_ready(agent) == False or abs(agent.ball.location.data[1]) > 5050:
            self.expired = True
        return agent.controller(agent, target_location, speed)


class QuickShot:
    def __init__(self):
        self.expired = False

    def available(self, agent):
        if ball_project(agent) > -(distance_2d(agent.ball, agent.me) / 2):
            return True
        return False

    def execute(self, agent):
        agent.controller = shot_controller
        left_post = Vector3([sign(agent.team) * goal_width / 2, -sign(agent.team) * field_length / 2, 100])
        right_post = Vector3([-sign(agent.team) * goal_width / 2, -sign(agent.team) * field_length / 2, 100])

        ball_left = angle2(agent.ball.location, left_post)
        ball_right = angle2(agent.ball.location, right_post)

        our_left = angle2(agent.me.location, left_post)
        our_right = angle2(agent.me.location, right_post)

        offset = (agent.ball.location.data[0] / field_width) * 3.14
        x = agent.ball.location.data[0] + 90 * abs(math.cos(offset)) * sign(offset)
        y = agent.ball.location.data[1] + 90 * abs(math.sin(offset)) * sign(agent.team)
        target_location = return_location([x, y, agent.ball.location.data[2]])

        location = return_local_location(target_location, agent.me)
        angle_to_target = math.atan2(location.data[1], location.data[0])
        distance_to_target = distance_2d(agent.me, target_location)

        speed_correction = ((2 + abs(angle_to_target) ** 2) * 350)
        speed = 2400 - speed_correction

        if self.available(agent) == False:
            self.expired = True
        elif CalcShot().available(agent) == True:
            self.expired = True

        return agent.controller(agent, target_location, speed)


class Wait():
    def __init__(self):
        self.expired = False

    def available(self, agent):
        if time_z(agent.ball) > 1.5:
            return True

    def execute(self, agent):
        # taking a rough guess at where the ball will be in the future, based on how long it will take to hit the ground
        ball_future = future(agent.ball, time_z(agent.ball))

        if agent.me.boost < 35:  # if we are low on boost, we'll go for boot
            closest = 0
            closest_distance = distance_2d(boosts[0], ball_future)

            # going through every large pad to see which one is closest to our ball_future guesstimation
            for i in range(1, len(boosts)):
                if distance_2d(boosts[i], ball_future) < closest_distance:
                    closest = i
                    closest_distance = distance_2d(boosts[i], ball_future)

            target = boosts[closest]
            speed = 2300
        else:
            # if we have boost, we just go towards the ball_future position, and slow down just like in ExampleATBA as we get close
            target = ball_future
            current = velocity_2d(agent.me)
            ratio = distance_2d(agent.me, target) / (current + 0.01)

            speed = cap(600 * ratio, 0, 2300)
        if speed <= 100:
            speed = 0

        if ball_ready(agent):
            self.expired = True

        return frugal_controller(agent, target, speed)


def frugal_controller(agent, target, speed):
    controller_state = SimpleControllerState()
    location = return_local_location(target, agent.me)
    angle_to_target = math.atan2(location.data[1], location.data[0])

    controller_state.steer = steer(angle_to_target)

    current_speed = velocity_2d(agent.me)
    if current_speed < speed:
        controller_state.throttle = 1.0
    elif current_speed - 50 > speed:
        controller_state.throttle = -1.0
    else:
        controller_state.throttle = 0

    if distance_2d(agent.me.location.data, target) < 250:
        controller_state.handbrake = 1

    # time_difference = time.time() - agent.start
    # if time_difference > 2.2 and distance_2d(target, agent.me) > (velocity_2d(agent.me) * 2.3) and abs(
    #         angle_to_target) < 1 and current_speed < speed:
    #     agent.start = time.time()
    # elif time_difference <= 0.1:
    #     controller_state.jump = True
    #     controller_state.pitch = -1
    # elif time_difference >= 0.1 and time_difference <= 0.15:
    #     controller_state.jump = False
    #     controller_state.pitch = -1
    # elif time_difference > 0.15 and time_difference < 1:
    #     controller_state.jump = True
    #     controller_state.yaw = controller_state.steer
    #     controller_state.pitch = -1

    return controller_state


def calc_controller(agent, target_object, target_speed):
    location = return_local_location(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1], location.data[0])

    current_speed = velocity_2d(agent.me)
    controller_state.steer = steer(angle_to_ball)

    # throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = -1.0
    return controller_state


def shot_controller(agent, target_object, target_speed):
    goal_local = return_local_location([0, -sign(agent.team) * field_length / 2, 100], agent.me)
    goal_angle = math.atan2(goal_local.data[1], goal_local.data[0])

    location = return_local_location(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_target = math.atan2(location.data[1], location.data[0])

    current_speed = velocity_2d(agent.me)
    # steering
    controller_state.steer = steer(angle_to_target)

    # throttle
    if target_speed > 1400 and target_speed > current_speed and agent.start > 2.5 and current_speed < 2250:
        controller_state.boost = True
    if target_speed > current_speed:
        controller_state.throttle = 1.0
    elif target_speed < current_speed:
        controller_state.throttle = 0

    # dodging
    time_difference = time.time() - agent.start
    if ball_ready(agent) and time_difference > 2.2 and distance_2d(target_object, agent.me) <= 270:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = math.sin(goal_angle)
        controller_state.pitch = -abs(math.cos(goal_angle))

    return controller_state


class ExampleATBA:
    def __init__(self):
        self.expired = False

    def execute(self, agent):
        target_location = agent.ball
        target_speed = velocity_2d(agent.ball) + (distance_2d(agent.ball, agent.me) / 1.5)

        return agent.controller(agent, target_location, target_speed)


def example_controller(agent, target_object, target_speed):
    location = return_local_location(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1], location.data[0])

    current_speed = velocity_2d(agent.me)
    # steering
    controller_state.steer = steer(angle_to_ball)

    # throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    # dodging
    time_difference = time.time() - agent.start
    if time_difference > 2.2 and distance_2d(target_object, agent.me) > (velocity_2d(agent.me) * 2.5) and abs(
            angle_to_ball) < 1.3:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = controller_state.steer
        controller_state.pitch = -1


    return controller_state

def turtle_controller(agent, target_object, target_speed):
    location = return_local_location(target_object, agent.me)
    controller_state = SimpleControllerState()
    angle_to_ball = math.atan2(location.data[1], location.data[0])

    current_speed = velocity_2d(agent.me)

    #turtle
    if abs(agent.bot_roll) < math.pi/2.5:
        controller_state.jump = True
        if agent.bot_roll >= 0:
            controller_state.roll = 1
        else:
            controller_state.roll = -1
    # steering
    controller_state.steer = steer(angle_to_ball)

    # throttle
    if target_speed > current_speed:
        controller_state.throttle = 1.0
        if target_speed > 1400 and agent.start > 2.2 and current_speed < 2250:
            controller_state.boost = True
    elif target_speed < current_speed:
        controller_state.throttle = 0

    # dodging
    time_difference = time.time() - agent.start
    if time_difference > 2.2 and distance_2d(target_object, agent.me) > (velocity_2d(agent.me) * 2.5) and abs(
            angle_to_ball) < 1.3:
        agent.start = time.time()
    elif time_difference <= 0.1:
        controller_state.jump = True
        controller_state.pitch = -1
    elif time_difference >= 0.1 and time_difference <= 0.15:
        controller_state.jump = False
        controller_state.pitch = -1
    elif time_difference > 0.15 and time_difference < 1:
        controller_state.jump = True
        controller_state.yaw = controller_state.steer
        controller_state.pitch = -1


    return controller_state