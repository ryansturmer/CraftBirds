import threading, socket, select, traceback,time
from math import floor
from Queue import Queue, Empty
import sqlite3

DB_PATH = 'craft_client.db'
LOG_PATH = 'log.txt'
COMMIT_INTERVAL = 5
CHUNK_SIZE=32
YOU = 'U'
BLOCK = 'B'
CHUNK = 'C'
POSITION = 'P'
DISCONNECT = 'D'
TALK = 'T'
KEY = 'K'
NICK = 'N'

def thread(f):
    def run(*k, **kw):
        t = threading.Thread(target=f, args=k, kwargs=kw)
        t.setDaemon(True)
        t.start()
        return t
    return run

def chunked(x):
    return int(floor(round(x) / CHUNK_SIZE))

class World(object):
    def __init__(self):
        self.cache = {}
        self.run()

    @thread
    def run(self):
        self.connection = sqlite3.connect(DB_PATH)
        self.create_tables()
        self.commit()
        while True:
            try:
                if time.time() - self.last_commit > COMMIT_INTERVAL:
                    self.commit()
            except Exception:
                traceback.print_exc()
                raise
    
    def commit(self):
        self.last_commit = time.time()
        self.connection.commit()
    
    def execute(self, *args, **kwargs):
        return self.connection.execute(*args, **kwargs)

    def create_tables(self):
        queries = [
            'create table if not exists block ('
            '    p int not null,'
            '    q int not null,'
            '    x int not null,'
            '    y int not null,'
            '    z int not null,'
            '    w int not null'
            ');',
            'create index if not exists block_xyz_idx on block (x, y, z);',
            'create unique index if not exists block_pqxyz_idx on '
            '    block (p, q, x, y, z);',
        ]
        for query in queries:
            self.execute(query)

class Player(object):
    def __init__(self, client):
        self.client = client
        self.position = (0.0,0.0,0.0,0.0,0.0)
        self.nick = ''
        self.id = None

    def __str__(self):
        if self.ready:
            x,y,z,rx,ry = self.position
            return "<Player id=%d '%s' at %0.3f, %0.3f, %0.3f, %0.3f, %0.3f>" % (self.id, self.nick, x,y,z,rx,ry)
        else:
            return "<Player Not Ready>"


    @property
    def ready(self):
        return self.id != None

    def command(self, command, data):
        self.talk('/%s %s\n' % (command, data))
    def talk(self, s):
        self.client.command(self, 'T,%s\n' % s)
    def set_nick(self, nick):
        self.nick = nick
        self.command('nick',nick)
    def set_position(self, x,y,z,rx,ry):
        self.position = (x,y,z,rx,ry)
        cmd = 'P,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n' % (x,y,z,rx,ry)
        self.client.command(self, cmd)
    def move(self, dx=0,dy=0,dz=0,drx=0,dry=0):
        x,y,z,rx,ry = self.position
        self.set_position(x+dx,y+dy,z+dz,rx+drx,ry+dry)

class Client(object):
    def __init__(self, host, port):
        self.lock = threading.RLock()
        #self.lock = FLock()
        self.host = host
        self.port = port
        self.sockets = {}
        self.players = {}
        self.queues = {}
        self.input_buffers = {}
        self.talk_handlers = []
        self.handlers = {
            'T' : self.on_talk,
            'K' : self.on_key,
            'B' : self.on_block,
            'C' : self.on_chunk,
            'U' : self.on_you,
            'P' : self.on_position,
            'N' : self.on_nick
        }
        self.opponents = {}
        self.opponent_positions = {}
        self.world = World()
        self.run()

    def add_talk_handler(self, handler):
        self.talk_handlers.append(handler)

    @property
    def ready(self):
        return all([player.ready for player in self.sockets])

    def get_player_by_id(self, id):
        with self.lock:
            for player in self.sockets:
                if player.id == id:
                    return player
            return None

    def get_player_by_nick(self, nick):
        with self.lock:
            for player in self.sockets:
                if player.nick == nick:
                    return player
            return None
    def add_player(self, nick=None):
        player = Player(self)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((self.host, self.port))
        s.setblocking(0)
        with self.lock:
            self.sockets[player] = s
            self.queues[player] = Queue()
            self.players[s] = player
            if nick:
                player.set_nick(nick)
        return player

    def remove_player(self, player):
        with self.lock:
            if player in self.sockets:
                s = self.sockets[player]
                del self.players[s]
                del self.sockets[player]
                del self.queues[player]

    def request_chunk(self,player,p,q,key):
        cmd = 'C,%d,%d,%d\n' % (p,q,key)
        self.command(player, cmd)

    def handle_command(self, player, command):
        args = command.strip().split(',')
        if args:
            cmd = args[0]
            handler = self.handlers.get(cmd, self.on_unhandled)
            handler(player, *args)

    def on_talk(self, player, *args):
        if self.talk_handlers:
            for handler in self.talk_handlers:
                handler(args[1])

    def on_position(self, player, *args):
        id = int(args[1])
        x,y,z,rx,ry = map(float, args[2:])
        if id in self.opponents:
            nick = self.opponents[id]
            self.opponent_positions[nick] = (x,y,z,rx,ry)

    def on_key(self, player, *args):
        pass

    def on_block(self, player, *args):
        pass

    def on_chunk(self, player, *args):
        pass
        
    def on_you(self, player, *args):
        id = int(args[1])
        position = map(float, args[2:])
        if len(position) == 5:
            player.id = id
            player.position = position

    def on_nick(self, player, *args):
        id = int(args[1])
        nick = str(args[2])
        player = self.get_player_by_id(id)
        if player:
            player.nick = nick
            return
        self.opponents[id] = nick


    def on_unhandled(self, player, *args):
        print "Unhandled command: %s, %s" % (player, str(args))

    @thread
    def run(self):
        while True:
            if self.players:
                readers, writers, errorers = select.select(self.sockets.values(), self.sockets.values(), self.sockets.values(), 60)
                # Deal with errors first
                for socket in errorers:
                    self.remove_player(self.players.get(socket, None))

                # Inbound: Data coming from server
                for socket in readers:
                    player = self.players.get(socket, None)
                    if player:
                        data = self.input_buffers.get(player, '') + socket.recv(4096)
                        lines = data.split('\n')
                        self.input_buffers[player] = lines[-1]
                        for line in lines[:-1]:
                            #print '<- %s' % line
                            self.handle_command(player, line)
                        
                # Outbound: Commands going to server
                for player, queue in self.queues.items():
                    try:
                        command = queue.get(False)
                        socket = self.sockets.get(player, None)
                        if socket and socket in writers:
                            #print '-> %s' % command,
                            socket.send(command)
                        queue.task_done()
                    except Empty:
                        pass
                    except Exception, e:
                        print e

    def command(self, player, cmd):
        self.queues[player].put(cmd)