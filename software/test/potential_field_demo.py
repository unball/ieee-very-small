#!/usr/bin/python

from __future__ import division
import math
import random
import time
import Tkinter

class Vector(object):
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return '%.2f, %.2f' % (self.x, self.y)

    def __add__(self, vector):
        return Vector(self.x + vector.x, self.y + vector.y)
    __radd__ = __add__

    def __sub__(self, vector):
        return Vector(self.x - vector.x, self.y - vector.y)
    __rsub__ = __sub__

    def __mul__(self, magnitude):
        return Vector(self.x * magnitude, self.y * magnitude)
    __rmul__ = __mul__

    def __truediv__(self, magnitude):
        return Vector(self.x / magnitude, self.y / magnitude)
    __rtruediv__ = __truediv__

    def __eq__(self, vector):
        return (self.x == vector.x and self.y == vector.y)

    @property
    def magnitude(self):
        return math.hypot(self.y, self.x)

    @property
    def angle(self):
        return math.atan2(self.y, self.x)

    def set_polar(self, magnitude, angle):
        self.x = magnitude * math.cos(angle)
        self.y = magnitude * math.sin(angle)

    def normalize(self):
        new_x = self.x / self.magnitude
        new_y = self.y / self.magnitude
        self.x = new_x
        self.y = new_y

class UnitVector(Vector):
    def __init__(self, angle):
        super(UnitVector, self).__init__(math.cos(angle), math.sin(angle))

class WorldObject(object):
    def __init__(self, position):
        self.position = position

class Field(WorldObject):
    def calculate_force(self, position):
        raise NotImplemented

class AttractionField(Field):
    magnitude = 1

    def __str__(self):
        return 'A'

    def calculate_force(self, position):
        position_diff = position - self.position
        force = Vector(0, 0)
        force.set_polar(-(AttractionField.magnitude - AttractionField.magnitude/(position_diff.magnitude + 1)), position_diff.angle)
        return force

class RepulsionField(Field):
    trigger_distance = 50
    magnitude = 20

    def __str__(self):
        return 'R'

    def calculate_force(self, position):
        position_diff = position - self.position
        force = Vector(0, 0)
        force.set_polar(RepulsionField.magnitude/(position_diff.magnitude + 1), position_diff.angle)
        return force

class Robot(WorldObject):
    num_robots = 0

    def __init__(self, position):
        super(Robot, self).__init__(position)
        self.id = Robot.num_robots
        Robot.num_robots += 1

    def __str__(self):
        return '%d' % self.id

class Cell(object):
    def __init__(self):
        self.objects = []

    def __str__(self):
        if len(self.objects) == 0:
            return '.'
        return str(self.objects[-1])

    def add(self, object):
        self.objects.append(object)

    def remove(self, object):
        self.objects = [o for o in self.objects if id(o) != id(object)]

class EmptyCell(Cell):
    def __init__(self):
        super(EmptyCell, self).__init__()

class Map(object):
    empty = None

    def __init__(self, width=1, height=1):
        self.objects = []
        self.cells = [[EmptyCell() for _ in range(width)] for _ in
                        range(height)]
        self.width = width
        self.height = height

    def __str__(self):
        string = []

        for cell_line in self.cells:
            for cell in cell_line:
                string.append(str(cell))
                string.append(' ')
            string.append('\n')

        return ''.join([str(line) for line in string])

    def add_object(self, object):
        self.objects.append(object)

    def move_object(self, object, to_position):
        self.objects.remove(object)
        object.position = to_position
        self.objects.append(object)

class World(object):
    def __init__(self, map_):
        self.robots = []
        self.fields = []
        self.map = map_
        self.goal = None

    def __str__(self):
        return str(self.map)

    def set_goal(self, position):
        self.goal = position

    def add_robot(self, position):
        robot = Robot(position)
        self.map.add_object(robot)
        self.robots.append(robot)
        return robot

    def add_attraction_field(self, position):
        field = AttractionField(position)
        self._add_field(field)
        return field

    def add_repulsion_field(self, position):
        field = RepulsionField(position)
        self._add_field(field)
        return field

    def _add_field(self, field):
        self.map.add_object(field)
        self.fields.append(field)

    def remove_field(self, field):
        self.fields.remove(field)

    def simulate(self):
        for robot in self.robots:
            force = Vector(0, 0)
            for field in self.fields:
                force += field.calculate_force(robot.position)

            new_position = robot.position + force

            if new_position.x >= self.map.width:
                new_position.x = self.map.width - 1
            elif new_position.x < 0:
                new_position.x = 0

            if new_position.y >= self.map.width:
                new_position.y = self.map.width - 1
            elif new_position.y < 0:
                new_position.y = 0

            self.map.move_object(robot, new_position)

    def is_finished(self):
        for robot in self.robots:
            if (robot.position - self.goal).magnitude < 1:
                return True
        return False

class Drawer(object):
    def __init__(self, width=100, height=100):
        self.master = Tkinter.Tk()
        self.canvas = Tkinter.Canvas(self.master, width=width, height=height)

    def draw_robot(self, position):
        width = 10
        height = 10
        x = position.x - width/2
        y = position.y - height/2
        self.canvas.create_rectangle(x, y, x + width, y + height, fill='black')

    def draw_attraction_field(self, position):
        width = 10
        height = 10
        x = position.x - width/2
        y = position.y - height/2
        self.canvas.create_oval(x, y, x + width, y + height, fill='green')

    def draw_repulsion_field(self, position):
        width = 10
        height = 10
        x = position.x - width/2
        y = position.y - height/2
        self.canvas.create_oval(x, y, x + width, y + height, fill='red')

    def draw_world(self, world):
        self.canvas.delete(Tkinter.ALL)

        for robot in world.robots:
            self.draw_robot(robot.position)

        for field in world.fields:
            if type(field) == AttractionField:
                self.draw_attraction_field(field.position)
            elif type(field) == RepulsionField:
                self.draw_repulsion_field(field.position)

        self.canvas.pack()
        self.master.update()

def main():
    world_map = Map(width=500, height=500)
    world = World(world_map)

    world.add_robot(Vector(2, 1))

    num_repulsion_fields = 10
    for _ in range(num_repulsion_fields):
        world.add_repulsion_field(Vector(random.randrange(world_map.width), random.randrange(world_map.height)))

    goal_field = world.add_attraction_field(Vector(random.randrange(world_map.width), random.randrange(world_map.height)))

    world.set_goal(goal_field.position)

    drawer = Drawer(width=world_map.width, height=world_map.height)

    while True:
        world.simulate()
        drawer.draw_world(world)
        time.sleep(0.003)

        if world.is_finished():
            world.remove_field(goal_field)
            goal_field = world.add_attraction_field(Vector(random.randrange(world_map.width), random.randrange(world_map.height)))
            world.set_goal(goal_field.position)
            print 'Finished'

if __name__ == '__main__':
    main()
