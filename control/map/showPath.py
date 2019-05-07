import pygame
import math
import argparse

ROAD_WIDTH = 9.0
IMG_WIDTH =  900
IMG_HEIGHT = 1000
PIXEL_DENSITY = 4
START_MAP_X = 25
START_MAP_Y = 25
FPS = 25



def main():
    road = load_routes()
    print(road[0])

    #init
    pygame.init()
    screen = pygame.display.set_mode( (IMG_WIDTH, IMG_HEIGHT), 0, 32)
    pygame.display.set_caption('show way path')
    fpsClock = pygame.time.Clock()
    screen.fill((0,0,0))

    map_points = []
    #edge_points = []
    for road_point in road:
        map_points.append(( int((road_point[0] + 45) * PIXEL_DENSITY) + 50, \
                            int((road_point[1] + 100) * PIXEL_DENSITY) + 50 ))
    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
            #pygame.draw.lines(screen, (255,255,255), True, map_points, int(ROAD_WIDTH * PIXEL_DENSITY) )
            #pygame.draw.polygon(screen, (255,255,255), map_points_right, 0 )
            #pygame.draw.polygon(screen, (0,0,0), map_points_left, 0 )
            #pygame.draw.polygon(screen, (255,255,255), edge_points, 0 )
            pygame.draw.lines(screen, (255,0,0), True, map_points, 5 )
            pygame.display.update()
            fpsClock.tick(FPS)
    finally:
        pygame.quit()
    pass


def load_routes():
    points = []

    def readfile(name):
        with open(name) as f:
            line = f.readline()
            while line:
                contents = line.split('\t')
                if len(contents) == 2:
                    x,y = float(contents[0]),  float(contents[1])
                    points.append(( x,y ))
                    pass
                line = f.readline()

    n = 'points.road'
    readfile(n)

    return points
    pass

if __name__ == '__main__':

    main()

    pass
