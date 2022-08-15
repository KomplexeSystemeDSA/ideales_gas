import numpy as np
import pygame

FPS = 60

class Atom:
    def __init__(self, pos: pygame.math.Vector2, vel: pygame.math.Vector2, mass: float, radius: float):
        self.pos = pos
        self.vel = vel
        self.mass = mass
        self.radius = radius

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, screen: pygame.Surface):
        pygame.draw.circle(screen, (255, 255, 255), self.pos, self.radius)

pygame.init()

screen = pygame.display.set_mode((500, 500))

running = True
clock = pygame.time.Clock()

atoms = [
    Atom(pygame.math.Vector2((10, 100)), pygame.math.Vector2((0.1, 0)), 1, 5),
    Atom(pygame.math.Vector2((300, 100)), pygame.math.Vector2((0, 0)), 1, 5)
]

while running:
    for evt in pygame.event.get():
        if evt.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))

    for a in atoms:
        for a2 in atoms:
            if a == a2:
                continue
            if a.pos.distance_to(a2.pos) <= a.radius + a2.radius:
                print("collision")
                o_vec = (a.pos - a2.pos).normalize()

                t_vec = o_vec.copy()
                new_y = t_vec.y * -1
                t_vec.x = t_vec.y
                t_vec.y = new_y
                

                a.vel = o_vec
                a2.vel = t_vec

        a.update(clock.tick(FPS))
        a.draw(screen)

    pygame.display.flip()