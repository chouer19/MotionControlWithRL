import pygame
import math
import argparse

ROAD_WIDTH = 9.0
IMG_WIDTH =  1000
IMG_HEIGHT = 1080
PIXEL_DENSITY = 5
START_MAP_X = 25
START_MAP_Y = 25
FPS = 25

def main():
    roads = load_routes()
    print(roads[0])

    #init
    pygame.init()
    screen = pygame.display.set_mode( (IMG_WIDTH, IMG_HEIGHT), 0, 32)
    pygame.display.set_caption('show way path')
    fpsClock = pygame.time.Clock()
    screen.fill((0,0,0))

    map_rects = []
    map_points = []
    #edge_points = []
    for road in roads:
        map_rects.append(pygame.Rect(int((road[0]+45) * PIXEL_DENSITY) + 20, int((road[1]+100) * PIXEL_DENSITY)+20, int(road[2] * PIXEL_DENSITY), int(road[3] * PIXEL_DENSITY) ))
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            #pygame.draw.lines(screen, (255,255,255), True, map_points, int(ROAD_WIDTH * PIXEL_DENSITY) )
            #pygame.draw.polygon(screen, (255,255,255), map_points_right, 0 )
            #pygame.draw.polygon(screen, (0,0,0), map_points_left, 0 )
            #pygame.draw.polygon(screen, (255,255,255), edge_points, 0 )
            for rect in map_rects:
                pygame.draw.rect(screen,(255,0,0),rect)
            pygame.display.update()
            fpsClock.tick(FPS)
    finally:
        pygame.quit()
    pass


def load_routes():
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
    readfile(n)

    return rect
    pass

if __name__ == '__main__':

    main()

    pass
