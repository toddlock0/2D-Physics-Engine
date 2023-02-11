import math


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))


class Point:
    def __init__(self, index, x, y, x_velocity, y_velocity, radius):
        self.index = index
        self.x = x
        self.y = y
        self.x_velocity = x_velocity
        self.y_velocity = y_velocity
        self.radius = radius


class Boundary:
    def __init__(self, x, y, height, width):
        self.x = x
        self.y = y
        self.height = height
        self.width = width
        self.max_radius = 0.0

    def contains(self, point):
        return self.x - self.width < point.x < self.x + self.width \
            and self.y - self.height < point.y < self.y + self.height

    def intersects(self, point):
        return self.x - self.width - self.max_radius - point.radius < point.x < self.x + self.width + self.max_radius + point.radius \
            and self.y - self.width - self.max_radius - point.radius < point.y < self.y + self.width + self.max_radius + point.radius

    def update_max_radius(self, radius):
        if self.max_radius < radius:
            self.max_radius = radius

    def string(self):
        return "(x {0}, y {1}, width {2}, height {3})".format(self.x, self.y, self.width, self.height)


class QuadTree:
    def __init__(self, boundary, capacity):
        self.boundary = boundary
        self.capacity = capacity
        self.points = []
        self.northeast = None
        self.northwest = None
        self.southeast = None
        self.southwest = None
        self.divided = False

    def subdivide(self):
        x = self.boundary.x
        y = self.boundary.y
        w = self.boundary.width
        h = self.boundary.height

        ne = Boundary(x + (w / 2.0), y + (h / 2.0), w / 2.0, h / 2.0)
        self.northeast = QuadTree(ne, self.capacity)
        nw = Boundary(x - (w / 2.0), y + (h / 2.0), w / 2.0, h / 2.0)
        self.northwest = QuadTree(nw, self.capacity)
        se = Boundary(x + (w / 2.0), y - (h / 2.0), w / 2.0, h / 2.0)
        self.southeast = QuadTree(se, self.capacity)
        sw = Boundary(x - (w / 2.0), y - (h / 2.0), w / 2.0, h / 2.0)
        self.southwest = QuadTree(sw, self.capacity)

        self.divided = True

    def insert(self, point):
        if not self.boundary.contains(point):
            return

        if len(self.points) < self.capacity:
            self.points.append(point)
            self.boundary.update_max_radius(point.radius)
        else:
            if not self.divided:
                self.subdivide()

            self.northeast.insert(point)
            self.northwest.insert(point)
            self.southeast.insert(point)
            self.southwest.insert(point)

    def check_this_for_collision(self, point, fpsi):
        earliest_collision_time = float('inf')
        earliest_collision_index = None

        for other_point in self.points:
            combined_radii = point.radius + other_point.radius
            if abs(point.x - other_point.x) <= combined_radii or abs(point.y - other_point.y) <= combined_radii:
                p = (point.x, point.y)
                d = ((point.x_velocity - other_point.x_velocity) * fpsi,
                     (point.y_velocity - other_point.y_velocity) * fpsi)
                q = (other_point.x, other_point.y)
                r = combined_radii

                d_dot_d = sum([i * j for (i, j) in zip(d, d)])
                d_dot_p = sum([i * j for (i, j) in zip(d, p)])
                d_dot_q = sum([i * j for (i, j) in zip(d, q)])
                p_dot_p = sum([i * j for (i, j) in zip(p, p)])
                q_dot_q = sum([i * j for (i, j) in zip(q, q)])
                p_dot_q = sum([i * j for (i, j) in zip(p, q)])

                a = d_dot_d
                b = 2.0 * (d_dot_p - d_dot_q)
                c = p_dot_p + q_dot_q - (2.0 * p_dot_q) - pow(r, 2)
                v_4ac = 4.0 * a * c

                # if b^2 < 4ac there is no solution
                if pow(b, 2) >= v_4ac:
                    time_of_collision = ((b * -1.0) - math.sqrt(pow(b, 2) - v_4ac)) / (2.0 * a)
                    # if time_of_collision is in interval [0, 1] there will be a collision
                    if 0.0 <= time_of_collision <= 1.0:
                        if time_of_collision < earliest_collision_time:
                            earliest_collision_time = time_of_collision
                            earliest_collision_index = other_point.index

        return earliest_collision_time, earliest_collision_index

    def check_collision(self, point, fpsi):
        if not self.boundary.intersects(point):
            return None, None

        earliest_collision_time, earliest_collision_index = self.check_this_for_collision(point, fpsi)

        if self.divided:
            ne_time, ne_index = self.northeast.check_collision(point, fpsi)
            nw_time, nw_index = self.northwest.check_collision(point, fpsi)
            se_time, se_index = self.southeast.check_collision(point, fpsi)
            sw_time, sw_index = self.southwest.check_collision(point, fpsi)

            if ne_time is not None and ne_time < earliest_collision_time:
                earliest_collision_time = ne_time
                earliest_collision_index = ne_index
            elif nw_time is not None and nw_time < earliest_collision_time:
                earliest_collision_time = nw_time
                earliest_collision_index = nw_index
            elif se_time is not None and se_time < earliest_collision_time:
                earliest_collision_time = se_time
                earliest_collision_index = se_index
            elif sw_time is not None and sw_time < earliest_collision_time:
                earliest_collision_time = sw_time
                earliest_collision_index = sw_index

        return earliest_collision_time, earliest_collision_index
