
import PyKDL as kdl
import pygame import msgs.pose_pb2 as GZ_pose

class Point(object):
    def __init__(self, x=0, y=0, yaw=0):
        self._x = x
        self._y = y
        self._yaw = yaw

    def x(self):
        return self._x

    def y(self):
        return self._y

    def yaw(self):
        return self._yaw

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

def roadContainsPose(box, pose):
    if pose.position.x >= box.X() and pose.position.x <= box.X() + box.Width():
        if pose.position.y >= box.Y() and pose.position.y <= box.Y() + box.Height():
            return True
    return False

def roadContainsBox(box, vehicle):
    if vehicle.X() >= box.X() and vehicle.X() <= box.X() + box.Width():
        if vehicle.Y() >= box.Y() and vehicle.Y() <= box.Y() + box.Height():
            return True
    if vehicle.X() + vehicle.Width() >= box.X() and vehicle.X() + vehicle.Width() <= box.X() + box.Width():
        if vehicle.Y() >= box.Y() and vehicle.Y() <= box.Y() + box.Height():
            return True
    if vehicle.X() + vehicle.Width() >= box.X() and vehicle.X() + vehicle.Width() <= box.X() + box.Width():
        if vehicle.Y() + vehicle.Height() >= box.Y() and vehicle.Y() + vehicle.Height() <= box.Y() + box.Height():
            return True
    if vehicle.X() + vehicle.Width() >= box.X() and vehicle.X() + vehicle.Width() <= box.X() + box.Width():
        if vehicle.Y() >= box.Y() and vehicle.Y() <= box.Y() + box.Height():
            return True
    return False

# reference point list of box to pos,
def BoxToPolygon(box, pos):
    points = []
    rot = kdl.Rotation.RotZ(-pos.orientation.x)
    point = kdl.Vector(box.X(),box.Y(),0)
    points.append(rot*point)
    point = kdl.Vector(box.X()+box.Width(),box.Y(),0)
    points.append(rot*point)
    point = kdl.Vector(box.X()+box.Width(),box.Y()+box.Height(),0)
    points.append(rot*point)
    point = kdl.Vector(box.X(),box.Y()+box.Height(),0)
    points.append(rot*point)
    return points

class Env(object):
    def __init__(self):
        self._roadBoxes = []
        self._vehicleBoxes = []
        self._trackPoints = []
        self._mark = 0
        self._pose = GZ_pose()
        self._box = Box()

        self._Width = 400
        self._Height = 320

        self._ImgCenter_X = 10
        self._ImgCenter_Y = 16

        self._PixelDensity = 10

        self._display = pygame.display.set_mode([self._Width,self._Height])
        pygame.init()
        pygame.display.set_caption("Motion Control using RL")

    def load_road(self, path):
        self._roadBoxes = []
        with open(path) as f:
            line = f.readline()
            contents = line.split(',')
            if len(contents) > 4:
                self._roadBoxes.append(Box(float(content[0]), float(content[1]), float(content[2]), float(content[3])))

    def road(self):
        return self._roadBoxes

    def load_track(self, path):
        self._trackPoints = []
        with open(path) as f:
            line = f.readline()
            contents = line.split(',')
            if len(contents) > 2:
                self._trackPoints.append(Point(float(content[0]), float(content[1]), float(content[2])))

    def track(self):
        return self._trackPoints

    def vehicle(self, path):
        return self._vehicles

    def _terminal(self, contacts, pose):
        for contact in contacts.contact:
            if 
            ("robocup_3Dsim_field_90::field::collision" not in contact.collision1) and
            ("robocup_3Dsim_field_90::field::collision" not in cantact.collision2):
                return True
        return False

    def _reward(self):
        reward = 0
        errorYaw = 999
        dis = 999
        for i in range(self._mark, self.mark+100):
            pass
        return reward

    def pose(self):
        return self._pose

    def render(self, contacts, pose):
        self._pose = pose
        roads = self.get_roads_polygon()
        vehicles = self.get_vehicles_polygon()

        # draw road
        # draw draw vehicles

        img = 0
        reward = 0
        return img, self._reward(), self._terminal()

    def get_roads_polygon(self):
        polygons = []
        roads = []
        for i,road in enumerate(self.road()):
            if roadContainsPose(road, self.pose()):
                roads.append(self.road()[i-1])
                roads.append(self.road()[i])
                roads.append(self.road()[i+1])
                roads.append(self.road()[i+2])
                break

        for road in roads:
            vectors = BoxToPolygon(road, self.pose())
            polygon = []
            polygon.append(( int((vectors[0].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[0].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[1].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[1].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[2].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[2].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[3].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[3].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygons.append(polygon)
        return polygons

    def get_vehicles_polygon(self):
        polygons = []
        vehicles = []
        for i,vehicle in enumerate(self.vehicle()):
            if vehicleContainsPose(vehicle, self.pose()):
                vehicles.append(self.vehicle()[i-1])
                vehicles.append(self.vehicle()[i])
                vehicles.append(self.vehicle()[i+1])
                vehicles.append(self.vehicle()[i+2])
                break

        for vehicle in vehicles:
            vectors = BoxToPolygon(vehicle, self.pose())
            polygon = []
            polygon.append(( int((vectors[0].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[0].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[1].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[1].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[2].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[2].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygon.append(( int((vectors[3].y()+self._ImgCenter_X)*self._PixelDensity), int((vectors[3].x()+self._ImageCenter_Y)*self.PixelDensity) ))
            polygons.append(polygon)
        return polygons

