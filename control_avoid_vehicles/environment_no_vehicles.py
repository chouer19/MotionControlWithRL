
import PyKDL as kdl
import pygame 
import msg.pose_pb2 as GZ_pose
import msg.vector3d_pb2 as GZ_vector3d
import math

class Env(object):
    def __init__(self, roadPath = None, vehiclePath = None, trackPath = None):
        self._roadBoxes = []
        self._containedRoadID = []
        self._vehicleBoxes = []
        self._trackPoints = []
        self._mark = 0
        self._pose = GZ_pose.Pose()
        self._speed = GZ_vector3d.Vector3d()
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
                print "collision"
                return True
        #for i in self._containedRoadID:
        roads = [self.road()[self._containedRoadID[0]],self.road()[self._containedRoadID[1]],self.road()[self._containedRoadID[2]],self.road()[self._containedRoadID[3]] ]
        if roadContainsSelf(roads ,self._pose) is False:
            print "out of road"
            return True
        return False

    def compute_ref_yaw(self,index):
        offset = 30
        tar_x = self.track()[(index+offset) % len(self.track())].x()
        tar_y = self.track()[(index+offset) % len(self.track())].y()
        self_x = self.pose().position.x
        self_y = self.pose().position.y
        vector_yaw = math.pi/2.0
        if tar_x == self_x:
            if self_y > tar_y:
                vector_yaw = math.pi / 2.0
            elif self_y == tar_y:
                vector_yaw = self.track()[(index+offset) % len(self.track())].yaw()
            elif self_y < tar_y:
                vector_yaw = math.pi * 3.0 / 2.0
        else:
            vector_yaw = math.atan((tar_y-self_y)/(tar_x-self_x))
            if tar_y - self_y < 0 and vector_yaw > 0:
                vector_yaw += math.pi
            if tar_y - self_y < 0 and vector_yaw < 0:
                vector_yaw += math.pi*2
            if tar_y - self_y > 0 and vector_yaw < 0:
                vector_yaw += math.pi
        return vector_yaw
        pass

    def _reward(self):
        dis = 9999
        index = 0
        loop = 0
        while True:
            dis = 9999
            loop += 1
            for i in range(self._mark, self._mark+200):
                tempDis =  ((self.track()[i % len(self.track())].x() - self.pose().position.x) ** 2 +
                (self.track()[i % len(self.track())].y() - self.pose().position.y) ** 2) ** 0.5 
                if dis > tempDis:
                    dis = tempDis
                    index = i % len(self.track())
            if dis > 6:
                self._mark = (self._mark + 200) % len(self.track())
            else:
                errorYaw = self.track()[(index+15) % len(self.track())].yaw() - self.pose().orientation.x
                errorYaw += self.track()[(index+6) % len(self.track())].yaw() - self.pose().orientation.x
                errorYaw /= 2
                errorYaw = self.track()[(index+20) % len(self.track())].yaw() - self.pose().orientation.x
                ref_yaw = self.compute_ref_yaw(index) + math.pi/2
                errorYaw2 = ref_yaw - self.pose().orientation.x
                #print(errorYaw)
                while errorYaw > math.pi:
                    errorYaw -= math.pi * 2
                while errorYaw < -1*math.pi:
                    errorYaw += math.pi * 2
                while errorYaw2 > math.pi:
                    errorYaw2 -= math.pi * 2
                while errorYaw2 < -1*math.pi:
                    errorYaw2 += math.pi * 2
                errorYaw = errorYaw2
                #errorYaw += errorYaw2
                #errorYaw = errorYaw * 0.5
                self._mark = index - 60
                #reward = max(math.cos(min(math.pi,max(-math.pi,errorYaw * 2.5))),0)
                reward = max(math.cos(min(math.pi,max(-math.pi,errorYaw * 3))),0)
                reward = math.cos(min(max(errorYaw * 2.3, -math.pi), math.pi) ) / ((dis/2+1)**2)
                #reward = reward/5
                #dis = dis + 2.5 * math.sin(abs(errorYaw))
                #if dis > 2.7:
                #    reward = math.exp(dis-2.5)/8
                #    reward = dis * -1 / 20
                #elif dis > 1.8:
                #    reward = reward * math.exp(3.6-2*dis)
                reward = round(reward / 3,3) 
                return reward, errorYaw
            if loop > 11:
                break
        #return -5, -5
        return -1, -1

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

    def render(self, contacts, pose, speed):
    #def render(self, terminal, pose, speed):
        self._pose = pose
        self._speed = speed
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
        velocity = (self._speed.x ** 2 + self._speed.y ** 2) ** 0.5
        velocity = (self._speed.x ** 2 + self._speed.y ** 2) * 0.5
        #reward = reward / 10
        #reward = self._reward() * velocity / 10
        reward , errorYaw = self._reward()
        reward = round(velocity / 8,3) * reward / 5
        reward = round(reward,3)
        reward = 0.06
        reward = 0.1
        if terminal:
            reward = -3
            reward = -5
            reward = -1
        return image_data, reward, terminal, self._intersection(), errorYaw

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

    def _intersection(self):
        sum = 0
        if roadContainsPose(self.road()[self._containedRoadID[0]], self.pose()):
            sum += 1
        if roadContainsPose(self.road()[self._containedRoadID[1]], self.pose()):
            sum += 1
        if sum > 1:
            return True
        if roadContainsPose(self.road()[self._containedRoadID[2]], self.pose()):
            sum += 1
        if sum > 1:
            return True
        if roadContainsPose(self.road()[self._containedRoadID[3]], self.pose()):
            sum += 1
        if sum > 1:
            return True
        return False

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
    if pose.position.x-pose.position.z/2 >= box.X() and pose.position.x-pose.position.z/2 <= box.X() + box.Width():
        if pose.position.y-pose.orientation.w/2 >= box.Y() and pose.position.y-pose.orientation.w/2 <= box.Y() + box.Height():
            return True
    if pose.position.x-pose.position.z/2 >= box.X() and pose.position.x-pose.position.z/2 <= box.X() + box.Width():
        if pose.position.y+pose.orientation.w/2 >= box.Y() and pose.position.y+pose.orientation.w/2 <= box.Y() + box.Height():
            return True
    if pose.position.x+pose.position.z/2 >= box.X() and pose.position.x+pose.position.z/2 <= box.X() + box.Width():
        if pose.position.y+pose.orientation.w/2 >= box.Y() and pose.position.y+pose.orientation.w/2 <= box.Y() + box.Height():
            return True
    if pose.position.x+pose.position.z/2 >= box.X() and pose.position.x+pose.position.z/2 <= box.X() + box.Width():
        if pose.position.y-pose.orientation.w/2 >= box.Y() and pose.position.y-pose.orientation.w/2 <= box.Y() + box.Height():
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
    rot = kdl.Rotation.RotZ(pos.orientation.x)
    point = rot*kdl.Vector(1.1, 2.3, 0)
    if not ((pos.position.x+point[0] >= box[0].X() and \
            pos.position.x+point[0] <= box[0].X()+box[0].Width() and \
            pos.position.y+point[1] >= box[0].Y() and \
            pos.position.y+point[1] <= box[0].Y()+box[0].Height()) or\

            (pos.position.x+point[0] >= box[1].X() and\
            pos.position.x+point[0] <= box[1].X()+box[1].Width() and \
            pos.position.y+point[1] >= box[1].Y() and \
            pos.position.y+point[1] <= box[1].Y()+box[1].Height()) or\

            (pos.position.x+point[0] >= box[2].X() and\
            pos.position.x+point[0] <= box[2].X()+box[2].Width() and \
            pos.position.y+point[1] >= box[2].Y() and \
            pos.position.y+point[1] <= box[2].Y()+box[2].Height()) or\

            (pos.position.x+point[0] >= box[3].X() and\
            pos.position.x+point[0] <= box[3].X()+box[3].Width() and \
            pos.position.y+point[1] >= box[3].Y() and \
            pos.position.y+point[1] <= box[3].Y()+box[3].Height())):
                print "point 1"
                return False
    point = rot*kdl.Vector(1.1, -2.3, 0)
    if not ((pos.position.x+point[0] >= box[0].X() and \
            pos.position.x+point[0] <= box[0].X()+box[0].Width() and \
            pos.position.y+point[1] >= box[0].Y() and \
            pos.position.y+point[1] <= box[0].Y()+box[0].Height()) or\

            (pos.position.x+point[0] >= box[1].X() and\
            pos.position.x+point[0] <= box[1].X()+box[1].Width() and \
            pos.position.y+point[1] >= box[1].Y() and \
            pos.position.y+point[1] <= box[1].Y()+box[1].Height()) or\

            (pos.position.x+point[0] >= box[2].X() and\
            pos.position.x+point[0] <= box[2].X()+box[2].Width() and \
            pos.position.y+point[1] >= box[2].Y() and \
            pos.position.y+point[1] <= box[2].Y()+box[2].Height()) or\

            (pos.position.x+point[0] >= box[3].X() and\
            pos.position.x+point[0] <= box[3].X()+box[3].Width() and \
            pos.position.y+point[1] >= box[3].Y() and \
            pos.position.y+point[1] <= box[3].Y()+box[3].Height())):
                print "point 2"
                return False
    point = rot*kdl.Vector(-1.1, -2.3, 0)
    if not ((pos.position.x+point[0] >= box[0].X() and \
            pos.position.x+point[0] <= box[0].X()+box[0].Width() and \
            pos.position.y+point[1] >= box[0].Y() and \
            pos.position.y+point[1] <= box[0].Y()+box[0].Height()) or\

            (pos.position.x+point[0] >= box[1].X() and\
            pos.position.x+point[0] <= box[1].X()+box[1].Width() and \
            pos.position.y+point[1] >= box[1].Y() and \
            pos.position.y+point[1] <= box[1].Y()+box[1].Height()) or\

            (pos.position.x+point[0] >= box[2].X() and\
            pos.position.x+point[0] <= box[2].X()+box[2].Width() and \
            pos.position.y+point[1] >= box[2].Y() and \
            pos.position.y+point[1] <= box[2].Y()+box[2].Height()) or\

            (pos.position.x+point[0] >= box[3].X() and\
            pos.position.x+point[0] <= box[3].X()+box[3].Width() and \
            pos.position.y+point[1] >= box[3].Y() and \
            pos.position.y+point[1] <= box[3].Y()+box[3].Height())):
                print "point 3"
                return False
    point = rot*kdl.Vector(-1.1, 2.3, 0)
    if not ((pos.position.x+point[0] >= box[0].X() and \
            pos.position.x+point[0] <= box[0].X()+box[0].Width() and \
            pos.position.y+point[1] >= box[0].Y() and \
            pos.position.y+point[1] <= box[0].Y()+box[0].Height()) or\

            (pos.position.x+point[0] >= box[1].X() and\
            pos.position.x+point[0] <= box[1].X()+box[1].Width() and \
            pos.position.y+point[1] >= box[1].Y() and \
            pos.position.y+point[1] <= box[1].Y()+box[1].Height()) or\

            (pos.position.x+point[0] >= box[2].X() and\
            pos.position.x+point[0] <= box[2].X()+box[2].Width() and \
            pos.position.y+point[1] >= box[2].Y() and \
            pos.position.y+point[1] <= box[2].Y()+box[2].Height()) or\

            (pos.position.x+point[0] >= box[3].X() and\
            pos.position.x+point[0] <= box[3].X()+box[3].Width() and \
            pos.position.y+point[1] >= box[3].Y() and \
            pos.position.y+point[1] <= box[3].Y()+box[3].Height())):
                print "point 4"
                return False
    return True

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

