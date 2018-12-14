import math
import time

from Util import *
from States import *

from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.utils.structures.game_data_struct import GameTickPacket

        
class MyBot(BaseAgent):

    def initialize_agent(self):
        #This runs once before the bot starts up
        self.me = obj()
        self.ball = obj()
        self.start = time.time()
        
        self.state = driveStraight()
        self.controller = driveStraightController
        
    #If state is expired, pick a new state
    def checkState(self):
        if self.state.expired == True:
            if driveStraight().available(self) == True:
                self.state = driveStraight()
            elif recover().available(self) == True:
                self.state = recover()
            elif calcShot().available(self) == True:
                self.state = calcShot()
            elif quickShot().available(self) == True:
                self.state = quickShot()
            else:
                self.state = quickShot()
                
                
# Function called 60 times/sec, takes in data and tells car what to do
    def get_output(self, game: GameTickPacket) -> SimpleControllerState:
        self.preprocess(game) # Convert game data into easy to manipulate format
        self.checkState() # Assign state if expired
        print(match_seconds_elapsed)
        return self.state.execute(self)
    
     

    
        
    # Renames location/velocity/etc. data from GameTickPacket
    def preprocess(self, game):
        self.me.location.data = [game.game_cars[self.index].physics.location.x,game.game_cars[self.index].physics.location.y,game.game_cars[self.index].physics.location.z]
        self.me.velocity.data = [game.game_cars[self.index].physics.velocity.x,game.game_cars[self.index].physics.velocity.y,game.game_cars[self.index].physics.velocity.z]
        self.me.rotation.data = [game.game_cars[self.index].physics.rotation.pitch,game.game_cars[self.index].physics.rotation.yaw,game.game_cars[self.index].physics.rotation.roll]
        self.me.rvelocity.data = [game.game_cars[self.index].physics.angular_velocity.x,game.game_cars[self.index].physics.angular_velocity.y,game.game_cars[self.index].physics.angular_velocity.z]
        self.me.matrix = rotator_to_matrix(self.me)
        self.me.boost = game.game_cars[self.index].boost
        self.me.has_wheel_contact = game.game_cars[self.index].has_wheel_contact

        self.ball.location.data = [game.game_ball.physics.location.x,game.game_ball.physics.location.y,game.game_ball.physics.location.z]
        self.ball.velocity.data = [game.game_ball.physics.velocity.x,game.game_ball.physics.velocity.y,game.game_ball.physics.velocity.z]
        self.ball.rotation.data = [game.game_ball.physics.rotation.pitch,game.game_ball.physics.rotation.yaw,game.game_ball.physics.rotation.roll]
        self.ball.rvelocity.data = [game.game_ball.physics.angular_velocity.x,game.game_ball.physics.angular_velocity.y,game.game_ball.physics.angular_velocity.z]

        self.ball.local_location = to_local(self.ball,self.me)
        
        match_seconds_elapsed = game.game_info.seconds_elapsed
         
