import math

goal_width = 1900
field_length = 10280
field_width = 8240

boosts = [
    [3584, 0, 0],
    [-3584, 0, 0],
    [3072, 4096, 0],
    [3072, -4096, 0],
    [-3072, 4096, 0],
    [-3072, -4096, 0],
]




class Vector3:
    def __init__(self, data):
        self.data = data

    def __add__(self, value):
        return Vector3([self.data[0] + value.data[0], self.data[1] + value.data[1], self.data[2] + value.data[2]])

    def __sub__(self, value):
        return Vector3([self.data[0] - value.data[0], self.data[1] - value.data[1], self.data[2] - value.data[2]])

    def __mul__(self, value):
        return (self.data[0] * value.data[0] + self.data[1] * value.data[1] + self.data[2] * value.data[2])

    def magnitude(self):
        return math.sqrt((self.data[0] * self.data[0]) + (self.data[1] * self.data[1]) + (self.data[2] * self.data[2]))

    def normalize(self):
        mag = self.magnitude()
        if mag != 0:
            return Vector3([self.data[0] / mag, self.data[1] / mag, self.data[2] / mag])
        else:
            return Vector3([0, 0, 0])


class Matrix2():
    def __init__(self, data):
        self.data = data

    def __mul__(self, value):
        return Vector2([value.data[0] * self.data[0][0] + value.data[1] * self.data[1][0],
                        value.data[0] * self.data[0][1] + value.data[1] * self.data[1][1]])


ROTATE = Matrix2([[0, -1], [1, 0]])


class Obj:
    def __init__(self):
        self.location = Vector3([0, 0, 0])
        self.velocity = Vector3([0, 0, 0])
        self.rotation = Vector3([0, 0, 0])
        self.rvelocity = Vector3([0, 0, 0])

        self.local_location = Vector3([0, 0, 0])
        self.boost = 0


def close_enough(object1, object2):
    if distance_2d(object1, object2) < 100:
        return True
    else:
        return False

def quad(a, b, c):
    inside = (b ** 2) - (4 * a * c)
    if inside < 0 or a == 0:
        return 0.1
    else:
        n = ((-b - math.sqrt(inside)) / (2 * a))
        p = ((-b + math.sqrt(inside)) / (2 * a))
        if p > n:
            return p
        return n


def future(ball, time):
    x = ball.location.data[0] + (ball.velocity.data[0] * time)
    y = ball.location.data[1] + (ball.velocity.data[1] * time)
    z = ball.location.data[2] + (ball.velocity.data[2] * time)
    return Vector3([x, y, z])


def time_z(ball):
    rate = 0.97
    return quad(-325, ball.velocity.data[2] * rate, ball.location.data[2] - 92.75)


def dpp(target_loc, target_vel, our_loc, our_vel):
    target_loc = return_location(target_loc)
    our_loc = return_location(our_loc)
    our_vel = return_location(our_vel)
    d = distance_2d(target_loc, our_loc)
    if d != 0:
        return (((target_loc.data[0] - our_loc.data[0]) * (target_vel.data[0] - our_vel.data[0])) + (
                    (target_loc.data[1] - our_loc.data[1]) * (target_vel.data[1] - our_vel.data[1]))) / d
    else:
        return 0


# def world_to_local(target_object, our_object):
#     x = (return_location(target_object) - our_object.location) * our_object.matrix[0]
#     y = (return_location(target_object) - our_object.location) * our_object.matrix[1]
#     z = (return_location(target_object) - our_object.location) * our_object.matrix[2]
#     return Vector3([x, y, z])

def return_local_location(target_object, our_object):
    if isinstance(target_object, Obj):
        return target_object.local_location
    else:
        x = (return_location(target_object) - our_object.location) * our_object.matrix[0]
        y = (return_location(target_object) - our_object.location) * our_object.matrix[1]
        z = (return_location(target_object) - our_object.location) * our_object.matrix[2]
        return Vector3([x, y, z])

def rotator_to_matrix(our_object):
    r = our_object.rotation.data
    cr = math.cos(r[2])
    sr = math.sin(r[2])
    cp = math.cos(r[0])
    sp = math.sin(r[0])
    cy = math.cos(r[1])
    sy = math.sin(r[1])

    matrix = []
    matrix.append(Vector3([cp * cy, cp * sy, sp]))
    matrix.append(Vector3([cy * sp * sr - cr * sy, sy * sp * sr + cr * cy, -cp * sr]))
    matrix.append(Vector3([-cr * cy * sp - sr * sy, -cr * sy * sp + sr * cy, cp * cr]))
    return matrix


def ball_ready(agent):
    ball = agent.ball
    if abs(ball.velocity.data[2]) < 150 and time_z(agent.ball) < 1:
        return True
    return False


def ball_project(agent):
    goal = Vector3([0, -sign(agent.team) * field_length / 2, 100])
    goal_to_ball = (agent.ball.location - goal).normalize()
    difference = agent.me.location - agent.ball.location
    return difference * goal_to_ball


def sign(x):
    if x <= 0:
        return -1
    else:
        return 1


def cap(x, low, high):
    if x < low:
        return low
    elif x > high:
        return high
    else:
        return x


def steer(angle):
    final = ((10 * angle + sign(angle)) ** 3) / 20
    return cap(final, -1, 1)


def angle_2(target_location, object_location):
    difference = return_location(target_location) - return_location(object_location)
    return math.atan2(difference.data[1], difference.data[0])


def velocity_2d(target_object):
    return math.sqrt(target_object.velocity.data[0] ** 2 + target_object.velocity.data[1] ** 2)



def return_location(target):
    if isinstance(target, Vector3):
        return target
    elif isinstance(target, list):
        return Vector3(target)
    else:
        return target.location


def distance_2d(target_object, our_object):
    difference = return_location(target_object) - return_location(our_object)
    return math.sqrt(difference.data[0] ** 2 + difference.data[1] ** 2)