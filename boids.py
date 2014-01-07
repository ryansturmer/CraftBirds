from client import Client, thread
import random, time
from numpy import array, add
from numpy.linalg import norm
from math import sin, cos, atan2, hypot, pi
import threading

HOST = '184.173.122.170'
PORT = 30624
LOC = (2380, 52, 4283)

#HOST = 'localhost'
#PORT = 4080
#LOC = (0,50,0)

with open('common.txt') as fp:
    BOID_NAMES = [name.strip() for name in fp.readlines() if ' ' not in name and "'" not in name]


BOID_INITIAL_POSITION = LOC
BOID_INITIAL_SPREAD = 30.0          # Boids start in a random cloud with this diameter
BOID_GRAVITY = 0.1                  # Force pulling boids inward towards their collective centers of mass
BOID_TARGET_GRAVITY = 0.1          # Flocking towards a target
BOID_PERSONAL_SPACE = 4.0           # How close a boid has to be to another boid to provoke it to start steering away
BOID_OBEDIENCE = 0.01              # Boid tendency to match speed with other boids in the flock
BOID_COUNT = 10                     # Number of boids in the flock
BOID_STEPS_PER_UPDATE = 20          # Simulation timestep
BOID_UPDATE_INTERVAL = 0.5          # Update interval
BOID_SCALE = 1.0                    # Scale factor that translates the simulated velocities
BOID_BOUNDS_CENTER = LOC            # Center of the boid boundary area
BOID_BOUNDS_X = 100                 # X extent of the boundary area
BOID_BOUNDS_Y = 35                  # Y extent of the boundary area
BOID_BOUNDS_Z = 100                 # Z extent of the boundary area
BOID_BOUNDS_REPULSION = 10.0        # How strongly boids are repelled from the boundaries
BOID_MAX_SPEED = 50.0               # Maximum speed of any boid
BOID_MIN_SPEED = 10.0               # Minimum speed of any boid
BOID_REPULSION = 1.0                # Boid repulsion from each other
BOID_USERNAME = 'birdmaster'

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
        self.gravity = BOID_GRAVITY
        self.flock_target = None
        self.bound = True
        self.bounds_center = BOID_BOUNDS_CENTER
        self.lock = threading.RLock()
    def set_update_function(self, func):
        self.update_function = func

    def update(self):
        if self.update_function:
            self.update_function(self)

    def setup_rules(self):
        self.rules = [  self.rule_center_of_mass,
                        self.rule_avoid_other_boids,
                        self.rule_match_velocities,
                        self.rule_limit_bounds,
                        self.rule_flock_target]

    def disperse(self):
        with self.lock:
            self.flock_target = None
            self.gravity = 0
            self.bound = True

    def regroup(self):
        with self.lock:
            self.bound = True
            self.flock_target = None
            self.gravity = BOID_GRAVITY

    def set_flock_target(self, v):
        with self.lock:
            self.disperse()
            self.flock_target = v
            self.bound = False

    def chillout(self):
        with self.lock:
            for boid in self.boids:
                boid.velocity /= 2.0

    def recenter(self, v):
        with self.lock:
            self.bounds_center = array(v)

    def run(self, iterations=None):
        # Evaluation means run a simulation timestep
        # Update means run the self.update_function to update the view
        self.evaluate()
        t = time.time()
        self.update()
        last_update = t
        i = 0
        while True:
            t1 = time.time()
            with self.lock:
                for step in range(BOID_STEPS_PER_UPDATE):
                    self.evaluate()
                    i+=1
            t2 = time.time()
            while t < last_update + BOID_UPDATE_INTERVAL:
                t = time.time()
            
            # Print time between updates / time it took to simulate
            #print '%s/%s' % (t - last_update, t2-t1) 
            self.update()
            last_update = t

            if iterations and  i >= iterations:
                break

    def evaluate(self):
        for boid in self.boids:
            # Apply all the rules in the model to this boid
            #vectors = [rule(boid) for rule in self.rules]
            dv = array([0,0,0])
            for rule in self.rules:
                dv += rule(boid)
            boid.velocity += dv
            # Enforce a speed limit
            speed = norm(boid.velocity)
            if speed > BOID_MAX_SPEED:
                boid.velocity = BOID_MAX_SPEED*boid.velocity/speed
            elif speed < BOID_MIN_SPEED:
                    boid.velocity = BOID_MIN_SPEED*boid.velocity/speed

            # Update the boid's position  
            boid.position = (boid.position + BOID_SCALE*boid.velocity/BOID_STEPS_PER_UPDATE)

    def rule_center_of_mass(self, me):
        'Velocity vector driving all boids to their collective center of mass'
        center_of_mass = array([0,0,0])
        for boid in self.boids:
            if boid is not me:
                center_of_mass =  center_of_mass + boid.position
        center_of_mass = center_of_mass/float(len(self.boids)-1)
        v = (center_of_mass - me.position)*self.gravity
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

    def rule_flock_target(self, me):
        if self.flock_target != None:
            return -(me.position - self.flock_target)*BOID_TARGET_GRAVITY
        else:
            return array([0,0,0])

    def rule_limit_bounds(self, me):
        'Nudge back in bounds if boid strays outside the bounding box'
        if not self.bound:
            return array([0,0,0])
        x,y,z = me.position
        dx,dy,dz = 0,0,0
        cx,cy,cz = self.bounds_center
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

    def command_handler(s):
        print 'handling command %s' % s
        try:
            s = s.lower().split()
            player = s[0].rstrip('>')
            target = s[1]
            cmd = s[2]
            args = s[3:]
            if player.startswith('bird') and target == 'birds':
                if cmd == 'disperse':
                    model.disperse()
                elif cmd == 'regroup':
                    model.regroup()
                elif cmd == 'attack':
                    nick = args[0]
                    pos = client.opponent_positions.get(nick, None)
                    if pos:
                        x,y,z,rx,ry = pos
                        print 'attacking target at %s' % (pos,)
                        model.set_flock_target(array([x,y,z]))
                elif cmd == 'come':
                    player = client.get_player_by_nick(player)
                    if player:
                        print 'recentering'
                        x,y,z,rx,ry = player.position
                        model.recenter(array([x,y,z]))
                elif cmd == 'chillout':
                    model.chillout()

        except Exception, e:
            print "Invalid command: '%s'" % s
            print e
    def update(model):
        for boid, player in zip(model.boids, client.players.values()):
            x,y,z = boid.position
            dx,dy,dz = boid.velocity
            rx,ry = v2r(dx,dy,dz)
            player.set_position(x,y,z,rx,ry)

    client.add_talk_handler(command_handler)
    model.set_update_function(update)
    
    # Wait for client to be ready
    while not client.ready:
        pass

    model.run()
