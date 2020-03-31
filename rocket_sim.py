import math 
import matplotlib.pyplot as plt
from collections import namedtuple, defaultdict
import numpy as np
import random
from treelib import Node, Tree
from pprint import pprint

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def rotate(self, angle):
        return Vector2(self.x * math.cos(angle) - self.y * math.sin(angle), self.x * math.sin(angle) + self.y * math.cos(angle))

    def times(self, value):
        return Vector2(self.x * value, self.y * value)

    @staticmethod
    def i():
        return Vector2(1, 0);

    @staticmethod
    def j():
        return Vector2(0, 1);


RocketPose = namedtuple('RocketPose', ['x', 'y', 'dx', 'dy', 'theta', 'dtheta', 'P', 'G'])

def pose_dist(pose1, pose2):
    return math.sqrt((pose2.x - pose1.x)**2 + (pose2.y - pose1.y)**2)

def weigthed_pose_dist(pose1, pose2):
    return (5 * (pose2.x - pose1.x)**2) + (pose2.y - pose1.y)**2

def pose_eq(pose1, pose2):
    return ((abs(pose2.x - pose1.x) <= 5) and \
            (abs(pose2.y - pose1.y) <= 5) and \
            (abs(pose2.dx - pose1.dx) <= 2) and \
            (abs(pose2.dy - pose1.dy) <= 2) and \
            (abs(pose2.theta - pose1.theta) <= 0.05) and \
            (abs(pose2.dtheta - pose1.dtheta) <= 0.1))

class RocketSimulation:
    def __init__(self, x, y, dx, dy, theta, dtheta, g=9.81, rOverI=0.15):
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.theta = theta
        self.dtheta = dtheta
        self.g = g
        self.rOverI = rOverI

    def __iter__(self):
        for i in range(self.n):
            yield self.update(self.iter_P, self.iter_G)

    def configure_iterator(self, n, P, G):
        self.n = n
        self.iter_P = P
        self.iter_G = G

    def clone(self):
        return RocketSimulation(self.x, self.y, self.dx, self.dy, self.theta, self.dtheta, self.g, self.rOverI)

    def update(self, P, G, dt=0.02):
        orientation = Vector2.j().rotate(-self.theta)
        thrust_vector = orientation.times(2 * self.g * P).rotate(-G)

        self.theta += self.dtheta * dt;
        self.dtheta += -(2 * self.g * P) * self.rOverI * math.sin(G) * dt

        self.x += self.dx * dt
        self.y += self.dy * dt
        self.dx += thrust_vector.x * dt
        self.dy += (-self.g * dt) + (thrust_vector.y * dt)

        return self.pose(P, G) 

    def update_n(self, P, G, n):
        return [self.update(P, G) for x in range(n)][-1]

    def pose(self, P, G):
        return RocketPose(self.x, self.y, self.dx, self.dy, self.theta, self.dtheta, P, G)


def find_path(startpose, endpose, path_step):

    #tree DST used to store path solutions.
    #each path is uniquely denoted by its leaf pointer.
    soln_tree = Tree()
    root = soln_tree.create_node(data=startpose) # root :)

    def path_from_leaf(node):
        path = [node.data]
        while not node.is_root(): 
            predecessor = node.predecessor(soln_tree.identifier)
            path.append(soln_tree.get_node(predecessor).data)
            node = soln_tree.get_node(predecessor)
        path.reverse()
        return path


    frontier = [root]
    next_frontier = []

    pass_ct = 0

    #let rocket start upside down, but remove cases where it's 
    #upside down once it's righted itself
    righted = False

    while frontier:
        pose_node = frontier.pop() 
        current_pose = pose_node.data

        def create_node(data):
            return soln_tree.create_node(data=data, parent=pose_node)

        if random.random() < (1 / 1000):
            path = path_from_leaf(pose_node)
            x = [s.x for s in path];
            y = [s.y for s in path];
            plt.plot(x, y, linewidth=2, color='blue')
            plt.savefig(f"plot{random.random()}.png")
            plt.close()


        #bruteforce with large set of P, G
        rs = RocketSimulation(current_pose.x, current_pose.y, current_pose.dx, 
                current_pose.dy, current_pose.theta, current_pose.dtheta) 
        next_states = [rs.clone().update_n(p, g, path_step) 
            for p in np.arange(0, 1, 0.05)
            for g in np.arange(-0.2, 0.2, 0.05)]

        def normalize_angle(angle):
            angle = angle % (2 * math.pi)
            angle = (angle + (2 * math.pi)) % (2 * math.pi)
            if angle > math.pi:
                angle -= (2 * math.pi)
            return angle

        def collision_filter(pose):
            L = 40
            W = 2
            s = math.sin(pose.theta)
            c = math.cos(pose.theta)

            outer_points = [
                Vector2(0, L/2),
                Vector2(1.8 * W, -(L/2) - W),
                Vector2(-1.8 * W, -(L/2) - W)
            ]

            for p in outer_points:
                if ((p.x * s) + (p.y * c) + pose.y) < 0:
                    return False
            return True

        def orientation_filter(pose):
            if abs(normalize_angle(pose.theta)) <= math.pi / 4:
                return True
            else:
                return not righted


        if not righted:
            for state in next_states:
                if abs(normalize_angle(state.theta)) <= math.pi / 4:
                    righted = True

        #filter states
        filters = [
            lambda pose: pose.y > 0,
            #lambda pose: pose_dist(startpose, endpose) > pose_dist(pose, endpose), #closer
            #lambda pose: abs(normalize_angle(pose.theta)) <= math.pi / 4, #upside-down bad
            collision_filter,
            orientation_filter
        ]
        
        
        next_states = list(filter(lambda x: all(f(x) for f in filters), next_states))

        correct_solns = list(filter(lambda pose: pose_eq(pose, endpose), next_states))

        if (correct_solns):
            print("Correct solutions found!!!")
            return [path_from_leaf(create_node(soln)) for soln in correct_solns]


        next_states = sorted(next_states, key=lambda x: pose_dist(x, endpose), 
                reverse=False) #try promising cases first

        if random.random() <= 1/ 500:
            print(f"Produced {len(next_states)} states during pass (t={pass_ct * path_step * 0.02})")
            if len(next_states) != 0:
                print(f"Closest distance: {pose_dist(next_states[0], endpose)}")


        if len(next_states) != 0:
            #always choose the best soln, also choose 2 random members
            next_frontier.append(create_node(next_states.pop()))
            #next_frontier.extend(
            #        [create_node(x) for x in random.sample(next_states[:30], 3)]
            #)
            next_frontier.extend(
                    [create_node(x) for x in random.sample(next_states, 
                        min(len(next_states), 2))]
            )


        if not frontier:
            if not next_frontier:
                print("No next frontier! Exiting!")

            frontier = next_frontier
            next_frontier = []
            print(f"Round {pass_ct} completed. Produced {len(frontier)} states.")
            pass_ct += 1 

            #generate summary plots
            if (pass_ct != 0) and (pass_ct % 15 == 0):
                all_paths = filter(
                    lambda x: len(x) == pass_ct + 1,
                    soln_tree.paths_to_leaves()
                )
                for identifier_list in all_paths:
                    node_list = [soln_tree.get_node(id) for id in identifier_list]
                    pose_list = [pose_node.data for pose_node in node_list]
                    x = [s.x for s in pose_list]
                    y = [s.y for s in pose_list]
                    plt.plot(x, y, linewidth=2, color='blue')

                plt.savefig(f"summary_plt{random.random()}.png")
                plt.close()

            #perform larger culling step in case of overfull frontier
            if len(frontier) > 4000:
                #Bucketization
                buckets = defaultdict(list)
                #bucket sizes
                bx, by, btheta = 2.5, 2.5, 0.5
                for f in frontier: 
                    buckets[hash((
                        math.floor(f.data.x / bx),
                        math.floor(f.data.y / by),
                        math.floor(f.data.theta / btheta)
                    ))].append(f)

                #pick representatives from buckets
                new_frontier = []
                for bucket, nodes in buckets.items():
                    #pick ideal and random candidate
                    sorted_poses = sorted(nodes, 
                            key=lambda node: weigthed_pose_dist(node.data, endpose))
                    new_frontier.append(sorted_poses[0])

                    if len(sorted_poses) > 1:
                        #make sure same candidate isnt chosen twice
                        sorted_poses.pop(0) 
                        new_frontier.extend(random.sample(sorted_poses, 
                            min(len(sorted_poses), 2)))

                        #re-add any exceptional cases
                        #for node in sorted_poses:
                        #    if node.data.y < 85:
                        #        new_frontier.append(node)

                print(f"Bucketization: removed {len(frontier) - len(new_frontier)}/{len(frontier)} elements")
                frontier = new_frontier



    print(f"frontier: {frontier}\nnext_frontier: {next_frontier}")
    return []

if __name__ == '__main__':
    rs = RocketSimulation(-100, 200, 0, -20, -0.3, 0)
    rs.configure_iterator(300, 1, 0.2);
    sim_values = [(res.x, res.y) for res in rs]
    x, y = zip(*sim_values)

    plt.plot(x, y, linewidth=2, color='blue')
    plt.show()

    path_inc = 50

    def write_path(path, name):
        sim_values = [(res.x, res.y) for res in path]
        x, y = zip(*sim_values)
        
        plt.plot(x, y, linewidth=2, color='blue')
        plt.savefig(f"path_{name}.png")

        path = path[1:]
        with open(f"path_{name}.rpth", "w") as f:
            f.write("let P = [\n")
            for pose in path:
                for i in range(path_inc):
                    f.write(f"{pose.P}, ")
                f.write("\n")
            f.write("];\n")
            f.write("let G = [\n")
            for pose in path:
                for i in range(path_inc):
                    f.write(f"{pose.G}, ")
                f.write("\n")
            f.write("];\n")
            

    paths = find_path(
        RocketPose(50, 200, 20, 60, 3.14, 0, 0, 0),
        RocketPose(0, 25, 0, 0, 0, 0, 0, 0),
        path_inc
    )
    print(paths)

    for index, path in enumerate(paths):
        write_path(path, index)


