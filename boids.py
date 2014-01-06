from client import Client, thread
import random, time
from numpy import array, add
from numpy.linalg import norm
from math import sin, cos, atan2, hypot, pi

HOST = '184.173.122.170'
PORT = 30624

#HOST = 'localhost'
#PORT = 4080

with open('common.txt') as fp:
    BOID_NAMES = [name.strip() for name in fp.readlines() if ' ' not in name and "'" not in name]

BOID_INITIAL_POSITION = (2380,52,4283)
BOID_INITIAL_SPREAD = 30.0          # Boids start in a random cloud with this diameter
BOID_GRAVITY = 0.001                # Force pulling boids inward towards their collective centers of mass
BOID_PERSONAL_SPACE = 4.0           # How close a boid has to be to another boid to provoke it to start steering away
BOID_OBEDIENCE = 0.125              # Boid tendency to match speed with other boids in the flock
BOID_COUNT = 10                     # Number of boids in the flock
BOID_TIMESTEP = 0.001               # Simulation timestep
BOID_UPDATE_INTERVAL = 0.5          # Update interval
BOID_SCALE = 0.03                   # Scale factor that translates the simulated velocities
BOID_BOUNDS_CENTER = (2380,70,4283) # Center of the boid boundary area
BOID_BOUNDS_X = 100                 # X extent of the boundary area
BOID_BOUNDS_Y = 35                  # Y extent of the boundary area
BOID_BOUNDS_Z = 100                 # Z extent of the boundary area
BOID_BOUNDS_REPULSION = 10.0        # How strongly boids are repelled from the boundaries
BOID_MAX_SPEED = 50.0               # Maximum speed of any boid
BOID_MIN_SPEED = 10.0               # Minimum speed of any boid
BOID_REPULSION = 1.0                # Boid repulsion from each other

class Boid(object):
    def __init__(self, position=None, velocity=None):
        self.position = array(position or [0,0,0])
        self.velocity = array(velocity or [0,0,0])

class BoidsModel(object):
    def __init__(self):
        self.boids = []
        self.rules = []
        self.update_function = None
        self.setup_rules()
        self.create_boids(BOID_COUNT)

    def set_update_function(self, func):
        self.update_function = func

    def update(self):
        if self.update_function:
            self.update_function(self)

    def setup_rules(self):
        self.rules = [  self.rule_center_of_mass,
                        self.rule_avoid_other_boids,
                        self.rule_match_velocities,
                        self.rule_limit_bounds]

    def run(self, iterations=None):
        # Evaluation means run a simulation timestep
        # Update means run the self.update_function to update the view
        last_eval = time.time() 
        last_update = last_eval
        self.update()
        self.evaluate()
        i = 0
        while True:
            t = time.time()
            if t > last_eval + BOID_TIMESTEP:
                self.evaluate()
                last_eval = t
                i += 1
            if t > last_update + BOID_UPDATE_INTERVAL:
                self.update()
                last_update = t
            if iterations and  i >= iterations:
                break

    def evaluate(self):
        for boid in self.boids:
            # Apply all the rules in the model to this boid
            vectors = [rule(boid) for rule in self.rules]
            dv = array([0,0,0])
            for v in vectors:
                dv = dv + v
            boid.velocity += dv

            # Enforce a speed limit
            speed = norm(boid.velocity)
            if speed > BOID_MAX_SPEED:
                boid.velocity = BOID_MAX_SPEED*boid.velocity/speed
            elif speed < BOID_MIN_SPEED:
                    boid.velocity = BOID_MIN_SPEED*boid.velocity/speed

            # Update the boid's position  
            boid.position = (boid.position + boid.velocity*BOID_SCALE)

    def rule_center_of_mass(self, me):
        'Velocity vector driving all boids to their collective center of mass'
        center_of_mass = array([0,0,0])
        for boid in self.boids:
            if boid is not me:
                center_of_mass =  center_of_mass + boid.position
        center_of_mass = center_of_mass/float(len(self.boids)-1)
        v = (center_of_mass - me.position)*BOID_GRAVITY
        return v

    def rule_avoid_other_boids(self, me):
        'Repulsion bump for when boids are too close'
        v = array([0,0,0])
        for boid in self.boids:
            if boid is not me:
                delta = boid.position - me.position
                dist = abs(norm(delta))
                if dist < BOID_PERSONAL_SPACE:
                    v = v - delta
        return v*BOID_REPULSION

    def rule_match_velocities(self, me):
        'Observe the velocities of other boids, and adjust speed to match gradually'
        v = array([0,0,0])
        for boid in self.boids:
            if boid is not me:
                v = v + boid.velocity
        v /= float(len(self.boids)-1)
        return BOID_OBEDIENCE*(v - me.velocity)

    def rule_limit_bounds(self, me):
        'Nudge back in bounds if boid strays outside the bounding box'
        x,y,z = me.position
        dx,dy,dz = 0,0,0
        cx,cy,cz = BOID_BOUNDS_CENTER
        if x < cx - BOID_BOUNDS_X:
            dx = BOID_BOUNDS_REPULSION
        elif x > cx + BOID_BOUNDS_X:
            dx = -BOID_BOUNDS_REPULSION
        if y < cy - BOID_BOUNDS_Y:
            dy = BOID_BOUNDS_REPULSION
        elif y > cy + BOID_BOUNDS_Y:
            dy = -BOID_BOUNDS_REPULSION
        if z < cz - BOID_BOUNDS_Z:
            dz = BOID_BOUNDS_REPULSION
        elif z > cz + BOID_BOUNDS_Z:
            dz = -BOID_BOUNDS_REPULSION
        return array([dx,dy,dz])

    def create_boids(self, count):
        # Start the boids with a little spread and an initial velocity to keep things interesting
        dx,dy,dz = random.uniform(50, 75.0),random.uniform(10.0, 20.0),random.uniform(50, 75.0)
        for i in range(count):
            x,y,z = BOID_INITIAL_POSITION
            x += random.uniform(-BOID_INITIAL_SPREAD/2.0, BOID_INITIAL_SPREAD/2.0)
            y += random.uniform(-BOID_INITIAL_SPREAD/2.0, BOID_INITIAL_SPREAD/2.0)
            z += random.uniform(-BOID_INITIAL_SPREAD/2.0, BOID_INITIAL_SPREAD/2.0) 
            self.boids.append(Boid(position=(x,y,z), velocity=(dx,dy,dz)))

# Thanks, Fogle!
def r2v(rx, ry):
    m = cos(ry)
    x = cos(rx - pi / 2) * m
    y = sin(ry)
    z = sin(rx - pi / 2) * m
    return (x, y, z)

def v2r(x, y, z):
    rx = atan2(z, x) + pi / 2
    ry = atan2(y, hypot(x, z)) 
    return (rx, ry)


if __name__ == '__main__':

    model = BoidsModel()
    client = Client(HOST, PORT)

    for boid in model.boids:
        name = random.choice(BOID_NAMES)
        client.add_player(name)

    def update(model):
        for boid, player in zip(model.boids, client.players.values()):
            x,y,z = boid.position
            dx,dy,dz = boid.velocity
            rx,ry = v2r(dx,dy,dz)
            player.set_position(x,y,z,rx,ry)

    model.set_update_function(update)
    
    # Wait for client to be ready
    while not client.ready:
        pass

    model.run()
