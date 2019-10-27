import pygame
import math
import argparse

ROAD_WIDTH = 9.0
IMG_WIDTH =  900
IMG_HEIGHT = 900
PIXEL_DENSITY = 4
START_MAP_X = 25
START_MAP_Y = 25
FPS = 25

def main():
    roads = load_routes("road.box")
    vehicles = load_routes("vehicle.box")

    #init
    pygame.init()
    screen = pygame.display.set_mode( (IMG_WIDTH, IMG_HEIGHT), 0, 32)
    pygame.display.set_caption('show way path')
    fpsClock = pygame.time.Clock()
    screen.fill((0,0,0))

    road_rects = []
    for road in roads:
        road_rects.append(pygame.Rect(int((road[0]+45) * PIXEL_DENSITY) + 20, int((100 - road[1] - road[3]) * PIXEL_DENSITY)+20, int(road[2] * PIXEL_DENSITY), int(road[3] * PIXEL_DENSITY) ))
        #road_rects.append(pygame.Rect(int((road[0]+45) * PIXEL_DENSITY) + 20, int((100 + road[1]) * PIXEL_DENSITY)+20, int(road[2] * PIXEL_DENSITY), int(road[3] * PIXEL_DENSITY) ))
    vehicle_rects = []
    for vehicle in vehicles:
        vehicle_rects.append(pygame.Rect(int((vehicle[0]+45) * PIXEL_DENSITY) + 20, int((100 - vehicle[1] - vehicle[3]) * PIXEL_DENSITY)+20, int(vehicle[2] * PIXEL_DENSITY), int(vehicle[3] * PIXEL_DENSITY) ))
        #vehicle_rects.append(pygame.Rect(int((vehicle[0]+45) * PIXEL_DENSITY) + 20, int((100 + vehicle[1]) * PIXEL_DENSITY)+20, int(vehicle[2] * PIXEL_DENSITY), int(vehicle[3] * PIXEL_DENSITY) ))
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            for rect in road_rects:
                pygame.draw.rect(screen,(255,0,0),rect)
            for rect in vehicle_rects:
                pygame.draw.rect(screen,(0,255,0),rect)
            pygame.display.update()
            fpsClock.tick(FPS)
    finally:
        pygame.quit()
    pass


def load_routes(path):
    rect = []

    def readfile(name):
        with open(name) as f:
            line = f.readline()
            while line:
                contents = line.split(',')
                if len(contents) >= 4:
                    x,y,width,height = float(contents[0]),  float(contents[1]),  float(contents[2]),  float(contents[3])
                    rect.append(( x,y,width,height ))
                    pass
                line = f.readline()

    n = 'road.box'
    n = 'vehicle.box'
    readfile(path)

    return rect
    pass

if __name__ == '__main__':

    main()

    pass
