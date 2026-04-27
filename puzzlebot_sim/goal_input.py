import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry


class MultiGoalInput(Node):
    def __init__(self):
        super().__init__('goal_input')

        # parametros basicos
        self.declare_parameter('robot_names', ['robot1', 'robot2'])
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('orientation_tolerance_deg', 2.0)
        self.declare_parameter('default_side_length', 1.0)

        self.robots = []
        for name in self.get_parameter('robot_names').value:
            self.robots.append(name.strip('/'))

        self.goal_tol = self.get_parameter('goal_tolerance').value
        deg_tol = self.get_parameter('orientation_tolerance_deg').value
        self.angle_tol = math.radians(deg_tol)
        self.default_side = self.get_parameter('default_side_length').value

        self.alias = {}
        self.pubs = {}
        self.pos = {}
        self.paths = {}
        self.lock = threading.Lock()

        for i, robot in enumerate(self.robots):
            self.alias[robot] = robot
            self.alias['r' + str(i + 1)] = robot

            self.pubs[robot] = self.create_publisher(
                Pose2D,
                '/' + robot + '/goal_pose',
                10
            )

            self.pos[robot] = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'ok': False}
            self.paths[robot] = {'active': False, 'points': [], 'i': 0}

            self.create_subscription(
                Odometry,
                '/' + robot + '/odom',
                lambda msg, r=robot: self.odom_cb(r, msg),
                10
            )

        self.create_timer(0.05, self.check_paths)

        hilo = threading.Thread(target=self.input_loop)
        hilo.daemon = True
        hilo.start()

    def fix_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def odom_cb(self, robot, msg):
        q = msg.pose.pose.orientation

        # convertir quaternion a yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta = math.atan2(siny, cosy)

        with self.lock:
            self.pos[robot]['x'] = msg.pose.pose.position.x
            self.pos[robot]['y'] = msg.pose.pose.position.y
            self.pos[robot]['theta'] = theta
            self.pos[robot]['ok'] = True

    def new_pose(self, x, y, theta_deg):
        p = Pose2D()
        p.x = float(x)
        p.y = float(y)
        p.theta = self.fix_angle(math.radians(float(theta_deg)))
        return p

    def send_goal(self, robot, pose):
        self.pubs[robot].publish(pose)
        print('%s -> x: %.2f, y: %.2f, theta: %.1f' % (
            robot, pose.x, pose.y, math.degrees(pose.theta)
        ))

    def read_pose(self, data):
        if len(data) != 3:
            raise ValueError('Usa: x y theta')
        return self.new_pose(data[0], data[1], data[2])

    def read_path(self, text):
        if text == '':
            raise ValueError('Usa: path x y th; x y th; ...')

        if ';' in text:
            partes = text.split(';')
        else:
            nums = text.split()
            if len(nums) % 3 != 0:
                raise ValueError('El path debe ir en grupos de 3 numeros')
            partes = []
            for i in range(0, len(nums), 3):
                partes.append(' '.join(nums[i:i + 3]))

        puntos = []
        for parte in partes:
            parte = parte.strip()
            if parte != '':
                puntos.append(self.read_pose(parte.split()))
        return puntos

    def arrived(self, robot, goal):
        with self.lock:
            p = dict(self.pos[robot])

        if not p['ok']:
            return False

        dx = goal.x - p['x']
        dy = goal.y - p['y']
        dist = math.sqrt(dx * dx + dy * dy)
        error_ang = self.fix_angle(goal.theta - p['theta'])

        return dist < self.goal_tol and abs(error_ang) < self.angle_tol

    def polygon(self, robot, sides, side):
        with self.lock:
            p = dict(self.pos[robot])

        if not p['ok']:
            raise ValueError('Todavia no llega odom de ' + robot)
        if side <= 0:
            raise ValueError('El lado debe ser mayor a cero')

        x = p['x']
        y = p['y']
        theta = p['theta']
        start_theta = theta
        turn = 2.0 * math.pi / sides
        points = []

        for i in range(sides):
            x = x + side * math.cos(theta)
            y = y + side * math.sin(theta)

            if i == sides - 1:
                theta_goal = start_theta
            else:
                theta_goal = self.fix_angle(theta + turn)

            pose = Pose2D()
            pose.x = x
            pose.y = y
            pose.theta = theta_goal
            points.append(pose)
            theta = theta_goal

        return points

    def start_path(self, robot, points, name):
        if len(points) == 0:
            raise ValueError('El path esta vacio')

        with self.lock:
            self.paths[robot]['active'] = True
            self.paths[robot]['points'] = points
            self.paths[robot]['i'] = 0

        print('Iniciando', name, 'en', robot)
        self.send_goal(robot, points[0])

    def stop_path(self, robot):
        with self.lock:
            self.paths[robot]['active'] = False
            self.paths[robot]['points'] = []
            self.paths[robot]['i'] = 0

    def check_paths(self):
        for robot in self.robots:
            with self.lock:
                path = dict(self.paths[robot])

            if not path['active']:
                continue

            i = path['i']
            points = path['points']

            if not self.arrived(robot, points[i]):
                continue

            i = i + 1
            if i >= len(points):
                self.stop_path(robot)
                print(robot + ' termino su path')
            else:
                with self.lock:
                    self.paths[robot]['i'] = i
                self.send_goal(robot, points[i])

    def get_robot(self, name):
        name = name.strip('/').lower()
        if name not in self.alias:
            raise ValueError('Robot no existe. Usa r1, r2, all o both')
        return self.alias[name]

    def print_help(self):
        print('\nCommands:')
        print('  r1 x y theta')
        print('  r2 x y theta')
        print('  both x1 y1 th1 x2 y2 th2')
        print('  all x y theta')
        print('  r1 path x y th; x y th; ...')
        print('  all path x y th; x y th; ...')
        print('  r1 square [lado]')
        print('  r2 pentagon [lado]')
        print('  all square [lado]')
        print('  all pentagon [lado]')
        print('  q\n')

    def do_robot(self, robot, parts, raw):
        if len(parts) >= 2 and parts[1] == 'path':
            text = raw.split(None, 2)
            if len(text) == 3:
                points = self.read_path(text[2])
            else:
                points = self.read_path('')
            self.start_path(robot, points, 'path')
            return

        if len(parts) >= 2 and parts[1] in ['square', 'pentagon']:
            side = self.default_side
            if len(parts) == 3:
                side = float(parts[2])

            if parts[1] == 'square':
                sides = 4
            else:
                sides = 5

            points = self.polygon(robot, sides, side)
            self.start_path(robot, points, parts[1])
            return

        pose = self.read_pose(parts[1:])
        self.stop_path(robot)
        self.send_goal(robot, pose)

    def do_all(self, parts, raw):
        if len(parts) >= 2 and parts[1] == 'path':
            text = raw.split(None, 2)
            if len(text) == 3:
                points = self.read_path(text[2])
            else:
                points = self.read_path('')

            for robot in self.robots:
                self.start_path(robot, list(points), 'path')
            return

        if len(parts) >= 2 and parts[1] in ['square', 'pentagon']:
            side = self.default_side
            if len(parts) == 3:
                side = float(parts[2])

            sides = 4 if parts[1] == 'square' else 5
            for robot in self.robots:
                points = self.polygon(robot, sides, side)
                self.start_path(robot, points, parts[1])
            return

        pose = self.read_pose(parts[1:])
        for robot in self.robots:
            self.stop_path(robot)
            self.send_goal(robot, pose)

    def do_both(self, parts):
        if len(self.robots) != 2:
            raise ValueError('both solo funciona con 2 robots')
        if len(parts) != 7:
            raise ValueError('Usa: both x1 y1 th1 x2 y2 th2')

        p1 = self.read_pose(parts[1:4])
        p2 = self.read_pose(parts[4:7])

        self.stop_path(self.robots[0])
        self.stop_path(self.robots[1])
        self.send_goal(self.robots[0], p1)
        self.send_goal(self.robots[1], p2)

    def input_loop(self):
        self.print_help()

        while rclpy.ok():
            try:
                raw = input('goal > ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if raw == '':
                continue
            if raw in ['q', 'quit', 'exit']:
                break
            if raw in ['h', 'help', '?']:
                self.print_help()
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            try:
                if cmd == 'all':
                    self.do_all(parts, raw)
                elif cmd == 'both':
                    self.do_both(parts)
                else:
                    robot = self.get_robot(cmd)
                    self.do_robot(robot, parts, raw)
            except ValueError as e:
                print(e)

        print('Cerrando goal_input')
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalInput()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
