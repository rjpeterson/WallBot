import math
import numpy as np

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket


class WallBot(BaseAgent):

    def initialize_agent(self):
        # This runs once before the bot starts up

        self.human = Car()
        self.bot = Car()
        self.ball = Ball()
        self.timer = 0.0


        self.kickoff_location = Vector2(0.0, 0.0)
        self.goalpost = Vector2(700, 5100)
        self.center_goal = Vector2(0, 5100)

        self.controller_state = SimpleControllerState()

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        self.preprocess(packet)
        self.return_ball_prediction(draw = True)

        if ball_prediction is not None:
            for i in range(0, ball_prediction.num_slices):
                prediction_slice = ball_prediction.slices[i]
                location = prediction_slice.physics.location
                self.logger.info("At time {}, the ball will be at ({}, {}, {})"
                                 .format(prediction_slice.game_seconds, location.x, location.y, location.z))

        own_center_goal = Vector2(self.center_goal.x, self.bot.team_sign * self.center_goal.y)
        own_negative_post = Vector2(-self.goalpost.x, self.bot.team_sign * self.goalpost.y)
        own_positive_post = Vector2(self.goalpost.x, self.bot.team_sign * self.goalpost.y)

        if self.ball.location == self.kickoff_location:
            target_location = self.ball.location
            drive_toward(self, target_location)
            self.controller_state.throttle = 1.0
            self.controller_state.boost = 1.0
            self.bot.state = "kickoff"
        else:
            if self.game_time - self.timer > 2.0:
                reset(self)
            else:
                pass
            # shadow the ball
            own_goal_relative_to_ball = own_center_goal - self.ball.location
            target_location = Vector2(self.ball.location.x  * 2/3, self.ball.location.y + own_goal_relative_to_ball.y * 1/5)

            drive_toward(self, target_location)
            self.bot.state = "drive"
            # relative_location = target_location - self.bot.location
            # steer_correction = self.bot.direction.correction_to(relative_location)
            # self.controller_state.steer = make_steer_correction(steer_correction)

            if distance_2d(self.ball.location, own_center_goal) < distance_2d(self.bot.location, own_center_goal) and self.bot.has_wheel_contact == True:
                self.controller_state.throttle = 1
                self.controller_state.boost = 1
                self.bot.state = "catchup"
            else:
                target_speed = distance_2d(self.bot.location, target_location)
                self.controller_state.throttle = match_target_velocity(self, target_speed)
                self.controller_state.boost = 0
                self.bot.state = "shadow"
            if self_to_ball_distance(self) < 700 and abs(self.bot.angle_to_ball) > 1.57:
                self.controller_state.handbrake = True
            if self_to_ball_distance(self) < 500:
                if abs(self.bot.angle_to_ball) > 1.57 or self.bot.halfflipping == True:
                    half_flip_toward_ball(self)
                    self.bot.state = "halfflip"
                else:
                    dodge_toward_ball(self)
                    self.bot.state = "dodge"

        self.renderer.begin_rendering()
        self.renderer.draw_line_3d([self.bot.location.x, self.bot.location.y, 20],
                                   [target_location.x, target_location.y, 20],
                                   self.renderer.create_color(255, 255, 0, 0))
        self.renderer.draw_string_2d(1, 1, 1, 1, "throttle = " + str(self.controller_state.throttle) + "\n" +
                                     "state = " + self.bot.state + "\n" +
                                     "jumped = " + str(self.bot.jumped) + "\n" +
                                     "wheelContact = " + str(self.bot.has_wheel_contact) + "\n" +
                                     "timerDiff = " + str(self.game_time - self.timer) + "\n" +
                                     "angleToBall = " + str(self.bot.angle_to_ball) + "\n" +
                                     "distanceToBall = " + str(self_to_ball_distance(self)) + "\n" +
                                     "flip pitch " + str(self.bot.flip_pitch) + "\n" +
                                     "flip yaw = " + str(self.bot.flip_yaw) + "\n" +
                                     "pitch = " + str(self.controller_state.pitch) + "\n" +
                                     "ballRelY = " + str(self.ball.relative_location.y) + "\n"
                                     ,self.renderer.create_color(255, 255, 255, 255))
        self.renderer.end_rendering()

        return self.controller_state

    def preprocess(self, packet):
        human = packet.game_cars[0].physics
        self.human.rotation = Vector2(human.rotation.pitch, human.rotation.yaw)
        self.human.location = Vector2(human.location.x, human.location.y)
        self.human.direction = Vector2(math.cos(human.rotation.pitch) * math.cos(human.rotation.yaw), math.cos(human.rotation.pitch) * math.sin(human.rotation.yaw))
        self.human.angle_to_ball = self.human.direction.correction_to(self.ball.location - self.human.location)
        self.human.has_wheel_contact = packet.game_cars[0].has_wheel_contact

        teams = {0: -1, 1: 1}
        bot = packet.game_cars[self.index]
        bot_pitch = float(bot.physics.rotation.pitch)
        bot_yaw = float(bot.physics.rotation.yaw)
        bot_roll = float(bot.physics.rotation.roll)

        self.bot.location = Vector2(bot.physics.location.x, bot.physics.location.y)
        self.bot.velocity = Vector2(bot.physics.velocity.x, bot.physics.velocity.y)
        self.bot.rotation = Vector2(bot_pitch, bot_yaw, bot_roll)
        self.bot.direction = Vector2(math.cos(bot_pitch) * math.cos(bot_yaw), math.cos(bot_pitch) * math.sin(bot_yaw))
        self.bot.angle_to_ball = self.bot.direction.correction_to(self.ball.relative_location)
        self.bot.team_sign = teams[bot.team]
        self.bot.jumped = bot.jumped
        self.bot.double_jumped = bot.double_jumped
        self.bot.has_wheel_contact = bot.has_wheel_contact

        ball = packet.game_ball
        self.ball.location = Vector2(ball.physics.location.x, packet.game_ball.physics.location.y)
        self.ball.velocity = Vector2(ball.physics.velocity.x, ball.physics.velocity.y)
        # ball location relative to bot
        self.ball.relative_location = self.ball.location - self.bot.location
        # ball location relative to bot orientation (probably wrong)
        self.ball.rel_loc_oriented = rotate2D(self.bot.angle_to_ball, self.ball.relative_location)

        self.game_time = packet.game_info.seconds_elapsed

    def return_ball_prediction(self, draw=False):
        self.ball_prediction_font = self.get_ball_prediction_struct()
        self.ball_prediction = [[], []]
        self.ball_prediction[0] = [None] * self.ball_prediction_font.num_slices
        self.ball_prediction[1] = [None] * self.ball_prediction_font.num_slices
        if self.ball_prediction_font is not None:
            for p_slice in range(0, self.ball_prediction_font.num_slices):
                prediction_slice = self.ball_prediction_font.slices[p_slice]
                prediction_loc = prediction_slice.physics.location
                self.ball_prediction[0][p_slice] = prediction_slice.game_seconds
                self.ball_prediction[1][p_slice] = la.vec3(
                    prediction_loc.x,
                    prediction_loc.y,
                    prediction_loc.z)
            if draw:
                self.renderer.begin_rendering('Prediction')
                self.renderer.draw_polyline_3d(self.ball_prediction[1], self.renderer.black())
                self.renderer.end_rendering()


class Car:
    def __init__(self):
        self.location = Vector2(0, 0)
        self.rotation = Vector2(0, 0)
        self.velocity = Vector2(0, 0)
        self.facing_vector = Vector2(0, 0)
        self.boost = 0
        self.has_dodge = True
        self.state = "drive"
        self.jumped = False
        self.halfflipping = False
        self.has_wheel_contact = True
        self.flip_start_time = 0.0
        self.flip_pitch = 0.0
        self.flip_yaw = 0.0


class Ball:
    def __init__(self):
        self.location = Vector2(0, 0)
        self.velocity = Vector2(0, 0)
        self.relative_location = Vector2(0, 0)


class Vector2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, val):
        return Vector2(self.x + val.x, self.y + val.y)

    def __sub__(self, val):
        return Vector2(self.x - val.x, self.y - val.y)

    def __str__(self):
        return str(self.x) + ", " + str(self.y)

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    def correction_to(self, ideal):
        # The in-game axes are left handed, so use -x
        current_in_radians = math.atan2(self.y, -self.x)
        ideal_in_radians = math.atan2(ideal.y, -ideal.x)

        correction = ideal_in_radians - current_in_radians

        # Make sure we go the 'short way'
        if abs(correction) > math.pi:
            if correction < 0:
                correction += 2 * math.pi
            else:
                correction -= 2 * math.pi

        return correction


def rotate2D(theta, vector):
    cs = math.cos(theta)
    sn = math.sin(theta)

    vx = vector.x * cs - vector.y * sn
    vy = vector.x * sn + vector.y * cs

    return Vector2(vx, vy)


def make_steer_correction(radians):
    """Takes desired steering correction in radians as input.
    Outputs controller_state.steer value of -1 or 1"""
    if radians > 0.1:
        # Positive radians in the unit circle is a turn to the left.
        return -1.0  # Negative value for a turn to the left.
    elif radians < -0.1:
        return 1.0
    else:
        return 0


def match_target_velocity(agent, target_speed):
    """Takes target object velocity in Unreal Units per second as input.
    Outputs controller_state.throttle value of 0 or 1"""
    if speed(target_speed) > speed(agent.bot.velocity):
        return 1
    else:
        return 0


def speed(target_velocity):
    """Takes object velocity vector and returns linear speed"""
    if isinstance(target_velocity, Vector2):
        return math.sqrt(target_velocity.x ** 2 + target_velocity.y ** 2)
    # if given speed as input, return speed
    elif target_velocity is float:
        return target_velocity
    else:
        return float(target_velocity)


def drive_toward(agent, target_location):
    relative_location = target_location - agent.bot.location
    steer_correction = agent.bot.direction.correction_to(relative_location)
    agent.controller_state.steer = make_steer_correction(steer_correction)


def distance_2d(obj1, obj2):
    return math.sqrt((obj1.x - obj2.x) ** 2 + (obj1.y - obj2.y) ** 2)

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def self_to_ball_distance(agent):
    return distance_2d(agent.bot.location, agent.ball.location)

def reset(agent):
    agent.bot.jumped = False
    agent.controller_state = SimpleControllerState()
    agent.bot.state = "drive"
    agent.bot.has_dodge = True

def dodge_toward_ball(agent):
    if agent.bot.jumped == False:
        agent.timer = agent.game_time
        agent.controller_state.jump = 1
        agent.bot.jumped = True
    elif agent.game_time - agent.timer < 0.05:
        agent.controller_state.jump = 0
        agent.controller_state.yaw = -math.sin(agent.bot.angle_to_ball)
        agent.controller_state.pitch = -math.cos(agent.bot.angle_to_ball)
    elif agent.game_time - agent.timer < 0.25:
        agent.controller_state.jump = 1
    elif agent.game_time - agent.timer < 1.0:
        agent.controller_state = SimpleControllerState()
        agent.bot.jumped = False


def half_flip_toward_ball(agent):
    output = agent.controller_state
    bot = agent.bot

    if bot.jumped == False:
        agent.timer = agent.game_time
        output.throttle = -1.0
        output.jump = True
        bot.jumped = True
        bot.halfflipping = True
        bot.flip_pitch = -math.cos(bot.angle_to_ball)
        bot.flip_yaw = -math.sin(bot.angle_to_ball)
    elif agent.game_time - agent.timer < 0.2:
        output.jump = False
    elif agent.game_time - agent.timer < 0.5:
        output.jump = True
        output.pitch = bot.flip_pitch
        output.roll = bot.flip_yaw
    elif agent.game_time - agent.timer < 1.0:
        output.boost = True
        output.throttle = True
        output.pitch = -bot.flip_pitch
        output.roll = -bot.flip_yaw
    elif agent.game_time - agent.timer >= 1.0:
        output = SimpleControllerState()
        bot.jumped = False
        bot.halfflipping = False

