import math

import numpy as np
import pygame

WIDTH = 1200
HEIGHT = 600
FPS = 60

class Atom:
    def __init__(self, pos: pygame.math.Vector2, vel: pygame.math.Vector2, mass: float, radius: float):
        self.pos = pos
        self.vel = vel
        self.mass = mass
        self.radius = radius

    def check_collision(self, other):
        distance_vec = other.pos - self.pos
        distance_vec_mag = distance_vec.magnitude()
        min_distance = self.radius + other.radius
        if distance_vec_mag >= min_distance:
            return False

        distance_correction = (min_distance - distance_vec_mag) / 2.0
        d = distance_vec.copy()
        correction_vector = d.normalize() * distance_correction
        other.pos += correction_vector
        self.pos -= correction_vector

        theta = math.atan2(distance_vec.x, distance_vec.y)
        sine = math.sin(theta)
        cosine = math.cos(theta)

        b_temp = [pygame.math.Vector2(), pygame.math.Vector2()]
        b_temp[1].x = cosine * distance_vec.x + sine * distance_vec.y
        b_temp[1].y = cosine * distance_vec.y - sine * distance_vec.x

        v_temp = [pygame.math.Vector2(), pygame.math.Vector2()]

        v_temp[0].x = cosine * self.vel.x + sine * self.vel.y
        v_temp[0].y = cosine * self.vel.y - sine * self.vel.x
        v_temp[1].x = cosine * other.vel.x + sine * other.vel.y
        v_temp[1].y = cosine * other.vel.y - sine * other.vel.x

        v_final = [pygame.math.Vector2(), pygame.math.Vector2()]

        v_final[0].x = ((self.mass - other.mass) * v_temp[0].x + 2 * other.mass * v_temp[1].x) / (self.mass + other.mass)
        v_final[0].y = v_temp[0].y

        v_final[1].x = ((other.mass - self.mass) * v_temp[1].x + 2 * self.mass * v_temp[0].x) / (self.mass + other.mass)
        v_final[1].y = v_temp[1].y

        b_temp[0].x += v_final[0].x
        b_temp[1].x += v_final[1].x

        b_final = [pygame.math.Vector2(), pygame.math.Vector2()]
        b_final[0].x = cosine * b_temp[0].x - sine * b_temp[0].x
        b_final[0].y = cosine * b_temp[0].y + sine * b_temp[0].x
        b_final[1].x = cosine * b_temp[1].x - sine * b_temp[1].y
        b_final[1].y = cosine * b_temp[1].y + sine * b_temp[1].x

        other.pos.x = self.pos.x + b_final[1].x
        other.pos.y = self.pos.y + b_final[1].y

        self.pos += b_final[0]

        self.vel.x = cosine * v_final[0].x - sine * v_final[0].y
        self.vel.y = cosine * v_final[0].y + sine * v_final[0].x
        other.vel.x = cosine * v_final[1].x - sine * v_final[1].y
        other.vel.y = cosine * v_final[1].y + sine * v_final[1].x

        return True

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, screen: pygame.Surface):
        pygame.draw.circle(screen, (255, 255, 255), self.pos, self.radius)

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))

running = True
clock = pygame.time.Clock()

"""atoms = [
    Atom(pygame.math.Vector2((10, 500)), pygame.math.Vector2((0.2, 0)), 1, 5),
    Atom(pygame.math.Vector2((500, 500)), pygame.math.Vector2((-0.1, 0)), 1, 5)
]"""
atoms = []
ATOM_MASS = 1
ATOM_RADIUS = 5
for i in range(1000):
    atoms.append(Atom(
        pygame.math.Vector2((np.random.randint(ATOM_RADIUS, WIDTH - ATOM_RADIUS, dtype=int), np.random.randint(ATOM_RADIUS, HEIGHT - ATOM_RADIUS, dtype=int))),
        pygame.math.Vector2(((np.random.rand() * 0.5) - 0.25, (np.random.rand() * 0.5) - 0.25)),
        ATOM_MASS,
        ATOM_RADIUS
    ))

while running:
    dt = clock.tick(FPS)

    for evt in pygame.event.get():
        if evt.type == pygame.QUIT:
            running = False

    screen.fill((0, 0, 0))

    for a in atoms:

        if a.pos.x <= a.radius or a.pos.x >= WIDTH - a.radius:
            a.vel.x *= -1
        if a.pos.y <= a.radius or a.pos.y >= HEIGHT - a.radius:
            a.vel.y *= -1

        black_list = []
        for a2 in atoms:
            if a == a2 or a2 in black_list:
                continue
            if a.check_collision(a2):
                black_list.append(a2)

        a.update(dt)
        a.draw(screen)

    pygame.display.flip()