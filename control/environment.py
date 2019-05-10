
import PyKDL as kdl
import pygame 
import msg.pose_pb2 as GZ_pose
import math

class Env(object):
    def __init__(self, roadPath = None, vehiclePath = None, trackPath = None):
        self._roadBoxes = []
        self._containedRoadID = []
        self._vehicleBoxes = []
        self._trackPoints = []
        self._mark = 0
        self._pose = GZ_pose.Pose()
        self._box = Box()

        self._Width = 400
        self._Height = 320

        self._ImgCenter_X = 10
        self._ImgCenter_Y = 16

        self._CarWidth = 2.2
        self._CarHeight = 4.6

        self._PixelDensity = 10

        self._display = pygame.display.set_mode([self._Width,self._Height])
        pygame.init()
        pygame.display.set_caption("Motion Control using RL")

        if vehiclePath is not None:
            self.load_vehicle(vehiclePath)

        if roadPath is not None:
            self.load_road(roadPath)

        if trackPath is not None:
            self.load_track(trackPath)

    def load_road(self, path):
        self._roadBoxes = []
        with open(path) as f:
            line = f.readline()
            while line:
                contents = line.split(',')
                line = f.readline()
                if len(contents) > 4:
                    self._roadBoxes.append(Box(float(contents[0]), float(contents[1]), float(contents[2]), float(contents[3])))

    def load_vehicle(self, path):
        self._vehicleBoxes = []
        with open(path) as f:
            line = f.readline()
            while line:
                contents = line.split(',')
                line = f.readline()
                if len(contents) > 4:
                    self._vehicleBoxes.append(Box(float(contents[0]), float(contents[1]), float(contents[2]), float(contents[3])))

    def road(self):
        return self._roadBoxes

    def load_track(self, path):
        self._trackPoints = []
        with open(path) as f:
            line = f.readline()
            while line:
                contents = line.split('\t')
                line = f.readline()
                if len(contents) > 2:
                    self._trackPoints.append(Point(float(contents[0]), float(contents[1]), math.pi/2 + float(contents[2])))

    def track(self):
        return self._trackPoints

    def vehicle(self):
        return self._vehicleBoxes

    def _terminal(self, contacts):
        for contact in contacts.contact:
            if ("prius_hybrid_123" not in contact.collision1) and ("prius_hybrid_123" not in contact.collision2):
                continue
            if ("robocup_3Dsim_field_90::field::collision" not in contact.collision1) and \
            ("robocup_3Dsim_field_90::field::collision" not in contact.collision2):
                return True
        for i in self._containedRoadID:
            if roadContainsSelf(self.road()[self._containedRoadID[0]], self._pose) is False:
                if roadContainsSelf(self.road()[self._containedRoadID[1]], self._pose) is False:
                    if roadContainsSelf(self.road()[self._containedRoadID[2]], self._pose) is False:
                        if roadContainsSelf(self.road()[self._containedRoadID[3]], self._pose) is False:
                            print "out of road"
                            return True
        return False

    def _reward(self):
        dis = 9999
        index = 0
        loop = 0
        while True:
            dis = 9999
            loop += 1
            for i in range(self._mark, self._mark+100):
                tempDis =  ((self.track()[i % len(self.track())].x() - self.pose().position.x) ** 2 +
                (self.track()[i % len(self.track())].y() - self.pose().position.y) ** 2) ** 0.5 
                if dis > tempDis:
                    dis = tempDis
                    index = i % len(self.track())
            if dis > 10:
                self._mark = (self._mark + 100) % len(self.track())
            else:
                errorYaw = self.track()[index % len(self.track())].yaw() - self.pose().orientation.x
                while errorYaw > math.pi:
                    errorYaw -= math.pi
                while errorYaw < -1*math.pi:
                    errorYaw += math.pi
                self._mark = index
                return (math.exp(-1 * abs(errorYaw)) - math.exp(-0.333 * math.pi)) / 8
            if loop > 11:
                break
        return -5

    def pose(self):
        return self._pose

    def testRender(self, pose):
        self._pose = pose
        roads = self.get_roads_polygon()
        vehicles = self.get_vehicles_polygon()
        # draw background
        pygame.draw.rect(self._display,(255,255,255), pygame.Rect((0,0),(self._Width, self._Height)))
        # draw road
        for road in roads:
            pygame.draw.polygon(self._display, (0,0,0), road, 0 )
        # draw self
        pygame.draw.rect(self._display,(0,0,255), pygame.Rect( (int((self._ImgCenter_X - self._CarHeight/2) *self._PixelDensity), int((self._ImgCenter_Y - self._CarWidth/2)*self._PixelDensity)), (int(self._CarHeight * self._PixelDensity),int(self._CarWidth * self._PixelDensity))))
        # draw draw vehicles
        for vehicle in vehicles:
            pygame.draw.polygon(self._display, (255,0,0), vehicle, 0 )
        pygame.display.flip()
        return self._reward()

    def render(self, contacts, pose):
        self._pose = pose
        roads = self.get_roads_polygon()
        vehicles = self.get_vehicles_polygon()
        # draw background
        pygame.draw.rect(self._display,(255,255,255), pygame.Rect((0,0),(self._Width, self._Height)))
        # draw road
        for road in roads:
            pygame.draw.polygon(self._display, (0,0,0), road, 0 )
        # draw self
        pygame.draw.rect(self._display,(0,0,255), pygame.Rect( (int((self._ImgCenter_X - self._CarHeight/2) *self._PixelDensity), int((self._ImgCenter_Y - self._CarWidth/2)*self._PixelDensity)), (int(self._CarHeight * self._PixelDensity),int(self._CarWidth * self._PixelDensity))))
        # draw draw vehicles
        for vehicle in vehicles:
            pygame.draw.polygon(self._display, (255,0,0), vehicle, 0 )
        pygame.display.flip()
        image_data = pygame.surfarray.array3d(pygame.display.get_surface())
        terminal = self._terminal(contacts)
        reward = self._reward()
        if terminal:
            reward = -5
        return image_data, reward, terminal

    def get_roads_polygon(self):
        polygons = []
        roads = []
        flag = True
        index = -1
        for i in self._containedRoadID:
            if roadContainsPose(self.road()[i], self.pose()):
                index = i
                flag = False
                break
        if flag:
            for i,road in enumerate(self.road()):
                if roadContainsPose(road, self.pose()):
                    index = i
                    flag = True 
                    break
        roads.append(self.road()[(index-1) % len(self.road())])
        roads.append(self.road()[index % len(self.road())])
        roads.append(self.road()[(index+1) % len(self.road())])
        roads.append(self.road()[(index+2) % len(self.road())])
        self._containedRoadID = []
        self._containedRoadID.append((index-1) % len(self.road()))
        self._containedRoadID.append((index) % len(self.road()))
        self._containedRoadID.append((index+1) % len(self.road()))
        self._containedRoadID.append((index+2) % len(self.road()))

        for road in roads:
            vectors = BoxToPolygon(road, self.pose())
            polygon = []
            polygon.append(( int((vectors[0].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[0].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[1].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[1].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[2].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[2].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[3].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[3].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygons.append(polygon)
        return polygons

    def get_vehicles_polygon(self):
        polygons = []
        vehicles = []
        for i,vehicle in enumerate(self.vehicle()):
            if roadContainsBox(self.road()[self._containedRoadID[0]], vehicle) or\
                    roadContainsBox(self.road()[self._containedRoadID[1]], vehicle) or\
                    roadContainsBox(self.road()[self._containedRoadID[2]], vehicle) or\
                    roadContainsBox(self.road()[self._containedRoadID[3]], vehicle):
                vehicles.append(self.vehicle()[i])

        for vehicle in vehicles:
            vectors = BoxToPolygon(vehicle, self.pose())
            polygon = []
            polygon.append(( int((vectors[0].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[0].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[1].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[1].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[2].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[2].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygon.append(( int((vectors[3].x()+self._ImgCenter_X)*self._PixelDensity),int((-vectors[3].y()+self._ImgCenter_Y)*self._PixelDensity) ))
            polygons.append(polygon)
        return polygons

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

    def Y(self):
        return self._y

    def Width(self):
        return self._width

    def Height(self):
        return self._height

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

def roadContainsSelf(box, pos):
    if pos.position.x-pos.position.z/2 > box.X() and pos.position.x-pos.position.z/2 < box.X()+box.Width():
        if pos.position.y-pos.orientation.w/2 > box.Y() and pos.position.y-pos.orientation.w/2 < box.Y()+box.Height():
            if pos.position.x+pos.position.z/2 > box.X() and pos.position.x+pos.position.z/2 < box.X()+box.Width():
                if pos.position.y-pos.orientation.w/2 > box.Y() and pos.position.y-pos.orientation.w/2 < box.Y()+box.Height():
                    if pos.position.x+pos.position.z/2 > box.X() and pos.position.x+pos.position.z/2 < box.X()+box.Width():
                        if pos.position.y+pos.orientation.w/2 > box.Y() and pos.position.y+pos.orientation.w/2 < box.Y()+box.Height():
                            if pos.position.x+pos.position.z/2 > box.X() and pos.position.x+pos.position.z/2 < box.X()+box.Width():
                                if pos.position.y-pos.orientation.w/2 > box.Y() and pos.position.y-pos.orientation.w/2 < box.Y()+box.Height():
                                    return True
    return False

# reference point list of box to pos,
def BoxToPolygon(box, pos):
    points = []
    rot = kdl.Rotation.RotZ(-pos.orientation.x+math.pi/2)
    point = kdl.Vector(box.X() - pos.position.x,box.Y() - pos.position.y,0)
    points.append(rot*point)
    point = kdl.Vector(box.X()+box.Width() - pos.position.x,box.Y() - pos.position.y,0)
    points.append(rot*point)
    point = kdl.Vector(box.X()+box.Width() - pos.position.x,box.Y()+box.Height() - pos.position.y,0)
    points.append(rot*point)
    point = kdl.Vector(box.X() - pos.position.x,box.Y()+box.Height() - pos.position.y,0)
    points.append(rot*point)
    return points

