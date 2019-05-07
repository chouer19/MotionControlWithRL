

import pygame import msgs.pose_pb2 as GZ_pose

class Box(object):
    def __init__(self, x=0,y=0,width=0,height=0):
        self._x = x
        self._y = y
        self._width = width
        self._height = height

    def X(self):
        return self._x

    def X(self,x):
        self._x = x

    def Y(self):
        return self._y

    def Y(self,y):
        self._y = y

    def Width(self):
        return self._width

    def Width(self,width):
        self._width = width

    def Height(self):
        return self._height

    def Height(self,height):
        self._height = height

def contains(box, pose):
    if pose.position.x >= box.X() and pose.position.x <= box.X() + box.Width():
        if pose.position.y >= box.Y() and pose.position.y <= box.Y() + box.Height():
            return True
    return False

class Env(object):
    def __init__(self):
        self._road = []
        self._vehicles = []
        self._pose = GZ_pose()
        self._box = Box()
        self._poseStack = []
        pass

    def load_road(self, path):
        pass

    def road(self):
        return self._road

    def vehicles(self, path):
        return self._vehicles

    def _terminal(self, contacts, pose):
        for contact in contacts.contact:
            if 
            ("robocup_3Dsim_field_90::field::collision" not in contact.collision1) and
            ("robocup_3Dsim_field_90::field::collision" not in cantact.collision2):
                return True
        return False

    def _reward(self, pose):
        reward = 0
        return reward
        pass

    def render(self, contacts, pose):

        img = 0
        reward = 0
        return img,reward,self._terminal()
