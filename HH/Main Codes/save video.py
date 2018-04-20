import pygame

import pygame.camera

from pygame.locals import *

pygame.init()

pygame.camera.init()

window = pygame.display.set_mode((640,480),0)

cam = pygame.camera.Camera(0)

cam.start()

image = cam.get_image()

pygame.image.save(image, 'abc.jpg')

cam.stop()