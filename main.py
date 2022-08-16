import math

import numpy as np
import pygame
import colorsys
import matplotlib.pyplot as plt
import collections
import time

PLOT = False

FULLSCREEN = True
WIDTH = 600
HEIGHT = 600
FPS = 60

ATOM_MASS = 1.0
ATOM_RADIUS = 50.0
NUM_ATOMS = 3

SPEED_FACTOR = 2.0

COLOR_FADE_DIVISOR = 1.0

#region Helper functions
def hsv2rgb(h: float, s: float, v: float):
    return tuple(round(i * 255) for i in colorsys.hsv_to_rgb(h, s, v))


def clamp(x, lower, higher):
    return max(lower, min(x, higher))
#endregion


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

        try:
            correction_vector = d.normalize() * distance_correction
        except ValueError:
            correction_vector = pygame.math.Vector2()

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
        if self.pos.x <= self.radius:
            self.pos.x = self.radius
            self.vel.x *= -1
        elif self.pos.x >= WIDTH - self.radius:
            self.pos.x = WIDTH - self.radius
            self.vel.x *= -1

        if self.pos.y <= self.radius:
            self.pos.y = self.radius
            self.vel.y *= -1
        elif self.pos.y >= HEIGHT - self.radius:
            self.pos.y = HEIGHT - self.radius
            self.vel.y *= -1

        self.pos += self.vel * dt

    def draw(self, screen: pygame.Surface):
        ms = time.time() % COLOR_FADE_DIVISOR / COLOR_FADE_DIVISOR
        hsv = (self.vel.magnitude() + ms) % 1.0, 1.0, 0.5
        rgb = hsv2rgb(*hsv)
        pygame.draw.circle(screen, rgb, self.pos, self.radius)


def main():
    global WIDTH, HEIGHT

    pygame.init()

    if FULLSCREEN:
        display_info = pygame.display.Info()
        WIDTH = display_info.current_w
        HEIGHT = display_info.current_h

    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.FULLSCREEN if FULLSCREEN else 0x0)

    running = True
    clock = pygame.time.Clock()

    atoms = []
    for i in range(NUM_ATOMS):
        atoms.append(Atom(
            pygame.math.Vector2((np.random.randint(ATOM_RADIUS, WIDTH - ATOM_RADIUS, dtype=int),
                                 np.random.randint(ATOM_RADIUS, HEIGHT - ATOM_RADIUS, dtype=int))),
            pygame.math.Vector2(((np.random.rand() * SPEED_FACTOR) - 0.5 * SPEED_FACTOR, (np.random.rand() * SPEED_FACTOR) - 0.5 * SPEED_FACTOR)),
            ATOM_MASS,
            ATOM_RADIUS
        ))

    while running:
        dt = clock.tick(FPS)

        #region Calculate velocity distribution
        vels = [a.vel.magnitude() for a in atoms]

        step = 2
        unit_vels = {}
        for v in vels:
            unit_vel = round(v, step)
            unit_vels[unit_vel] = unit_vels.get(unit_vel, 0) + 1
        #endregion

        for evt in pygame.event.get():
            if evt.type == pygame.QUIT:
                running = False
                if PLOT:
                    od = collections.OrderedDict(sorted(unit_vels.items()))
                    x_axis, y_axis = zip(*od.items())
                    plt.plot(x_axis, y_axis)
                    plt.show()

        screen.fill((0, 0, 0))

        for a in atoms:
            black_list = []
            for a2 in atoms:
                if a == a2 or a2 in black_list:
                    continue
                if a.check_collision(a2):
                    black_list.append(a2)

            a.update(dt)
            a.draw(screen)

        pygame.display.flip()


if __name__ == '__main__':
    main()