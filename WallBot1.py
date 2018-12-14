import math
import time
from Util import *
from States import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket



class WallBot(BaseAgent):

    def initialize_agent(self):
        self.me = Obj()
        self.ball = Obj()
        self.players = []  # holds other players in match
        self.start = time.time()

        self.state = MoveToPost()
        self.controller_state = SimpleControllerState()
        self.controller_state.throttle = 0

    def get_output(self, game: GameTickPacket) -> SimpleControllerState:
        self.preprocess(game)
        self.state = MoveToPost()

        # self.renderer.begin_rendering()
        # self.renderer.draw_string_2d(1, 1, 1, 1, str(distance_2d(self.me.location.data, self.target)), self.renderer.white())
        # self.renderer.end_rendering()

        return self.state.execute(self)

    def preprocess(self, game):
        self.players = []
        car = game.game_cars[self.index]
        self.me.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
        self.me.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
        self.me.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
        self.me.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y,
                                  car.physics.angular_velocity.z]
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.boost = car.boost
        self.me.team = car.team

        ball = game.game_ball.physics
        self.ball.location.data = [ball.location.x, ball.location.y, ball.location.z]
        self.ball.velocity.data = [ball.velocity.x, ball.velocity.y, ball.velocity.z]
        self.ball.rotation.data = [ball.rotation.pitch, ball.rotation.yaw, ball.rotation.roll]
        self.ball.rvelocity.data = [ball.angular_velocity.x, ball.angular_velocity.y, ball.angular_velocity.z]

        self.ball.local_location = return_local_location(self.ball, self.me)

        # collects info for all other cars in match, updates objects in self.players accordingly
        for i in range(game.num_cars):
            if i != self.index:
                car = game.game_cars[i]
                temp = Obj()
                temp.index = i
                temp.team = car.team
                temp.location.data = [car.physics.location.x, car.physics.location.y, car.physics.location.z]
                temp.velocity.data = [car.physics.velocity.x, car.physics.velocity.y, car.physics.velocity.z]
                temp.rotation.data = [car.physics.rotation.pitch, car.physics.rotation.yaw, car.physics.rotation.roll]
                temp.rvelocity.data = [car.physics.angular_velocity.x, car.physics.angular_velocity.y,
                                       car.physics.angular_velocity.z]
                self.me.boost = car.boost
                flag = False
                for item in self.players:
                    if item.index == i:
                        item = temp
                        flag = True
                        break
                if flag:
                    self.players.append(temp)





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
