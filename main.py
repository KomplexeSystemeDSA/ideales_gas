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

    def check_collision(self, other: Atom):
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
        """sum_mass = a.mass + a2.mass
        a.vel -= ((2 * a2.mass) / sum_mass) * (((a.vel - a2.vel) * (a.pos - a2.pos)) / (a.pos - a2.pos).magnitude_squared()) * (a.pos - a2.pos)
        a2.vel -= ((2 * a.mass) / sum_mass) * (
                    ((a2.vel - a.vel) * (a2.pos - a.pos)) / (a2.pos - a.pos).magnitude_squared()) * (
                             a2.pos - a.pos)"""
        distance_correction = ()

    def update(self, dt: float):
        self.pos += self.vel * dt

    def draw(self, screen: pygame.Surface):
        pygame.draw.circle(screen, (255, 255, 255), self.pos, self.radius)

pygame.init()

screen = pygame.display.set_mode((500, 500))

running = True
clock = pygame.time.Clock()

"""atoms = [
    Atom(pygame.math.Vector2((10, 500)), pygame.math.Vector2((0.2, 0)), 1, 5),
    Atom(pygame.math.Vector2((500, 500)), pygame.math.Vector2((-0.1, 0)), 1, 5)
]"""
atoms = []
ATOM_MASS = 1
ATOM_RADIUS = 5
for i in range(500):
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

        for a2 in atoms:
            if a == a2:
                continue
            a.check_collision(a2)

        a.update(dt)
        a.draw(screen)

    pygame.display.flip()