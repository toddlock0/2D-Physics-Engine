import tkinter
import time
import math
import random

# 1 seconds in nanoseconds
SECOND_IN_NS = 1000000000
FRAMES_PER_SECOND = 30.0
FRAMES_PER_SECOND_INVERTED = 1.0 / FRAMES_PER_SECOND
# Update rate in nanoseconds
UPDATE_RATE_NS = SECOND_IN_NS / FRAMES_PER_SECOND
# Sleep for 1/10th the update rate
SLEEP_TIME_S = 1.0 / (UPDATE_RATE_NS * 10.0)

NUMBER_BALLS = 20

GRAVITY_ACCELERATION = -9.81 * 40.0
CEILING_SPRING_CONSTANT = 100.0
GROUND_HEIGHT = 50.0
CLICK_SPRING_CONSTANT = 1000.0
ROLLING_FRICTION_COEFFICIENT = 0.99
ROLLING_FRICTION_COEFFICIENT_PER_SECOND = ROLLING_FRICTION_COEFFICIENT / FRAMES_PER_SECOND
BALL_DENSITY = 0.01

GRAVITY_ENABLED = True
SPRING_ENABLED = False
AIR_FRICTION_ENABLED = False

SPRING_X = 150
SPRING_Y = 300

MIN_X = 0.0
MAX_X = 1200.0
MIN_Y = 0.0
MAX_Y = 800.0

BALLS = []
BALL_MIN_RADIUS = 10
BALL_MAX_RADIUS = 40
BALL_STARTS = [(random.randrange(int(MAX_X - BALL_MAX_RADIUS)), random.randrange(int(MAX_Y - BALL_MAX_RADIUS))) for i in range(NUMBER_BALLS)]
BALL_COLORS = ["white", "red", "green", "blue", "yellow", "orange", "purple", "black", "cyan", "magenta"]
BALL_START_SPEED_MAX = 1.0

# globals
run = True
Clicked = False
ClickLocationX = 0
ClickLocationY = 0


def click(event):
    global Clicked, ClickLocationX, ClickLocationY

    Clicked = True
    ClickLocationX = event.x
    ClickLocationY = MAX_Y - event.y


def click_release(event):
    global Clicked
    Clicked = False


def main():
    # init tk
    root = tkinter.Tk()

    # create canvas
    canvas = tkinter.Canvas(root, bg="white", height=int(MAX_Y), width=int(MAX_X))

    def handler():
        global run
        run = False

    root.title("Physics Engine")
    root.protocol("WM_DELETE_WINDOW", handler)
    
    canvas.bind("<Button-1>", click)
    canvas.bind("<ButtonRelease-1>", click_release)

    fps_text = canvas.create_text(10, 5, fill="darkblue", font="Times 11 italic bold", anchor=tkinter.NW, text="FPS:")
    debug_text_1 = canvas.create_text(10, 19, fill="darkblue", font="Times 11 italic bold", anchor=tkinter.NW, text="")
    debug_text_2 = canvas.create_text(10, 33, fill="darkblue", font="Times 11 italic bold", anchor=tkinter.NW, text="")
    debug_text_3 = canvas.create_text(10, 47, fill="darkblue", font="Times 11 italic bold", anchor=tkinter.NW, text="")

    for i, (ball_x, ball_y) in enumerate(BALL_STARTS):
        color = BALL_COLORS[random.randrange(len(BALL_COLORS))]
        radius = ((BALL_MAX_RADIUS - BALL_MIN_RADIUS) * random.random()) + BALL_MIN_RADIUS
        x_velocity = random.uniform(0.0, BALL_START_SPEED_MAX)
        y_velocity = random.uniform(0.0, BALL_START_SPEED_MAX)
        BALLS.append([canvas.create_oval(ball_x - radius, MAX_Y - (ball_y - radius),
                                         ball_x + radius, MAX_Y - (ball_y + radius), fill=color),
                      x_velocity, y_velocity, radius])

    # add to window and show
    canvas.pack()

    current_time = time.time_ns()
    next_tick = current_time + UPDATE_RATE_NS
    next_second = current_time + SECOND_IN_NS
    frames = 0

    while run:
        # Update ball velocities for gravity, springs, and friction
        for index, (ball, x_velocity, y_velocity, radius) in enumerate(BALLS):
            ball_x, ball_y = ball_center(canvas.coords(ball))
            ball_volume = (4.0 / 3.0) * math.pi * radius * radius * radius
            ball_mass = ball_volume * BALL_DENSITY

            if GRAVITY_ENABLED:
                y_velocity += GRAVITY_ACCELERATION / FRAMES_PER_SECOND
            if SPRING_ENABLED:
                spring_distance = euclidean_distance(ball_x, ball_y, SPRING_X, SPRING_Y)
                spring_force = ((CEILING_SPRING_CONSTANT * spring_distance) / ball_mass)
                spring_angle_radians = angle_horizon(ball_x, ball_y, SPRING_X, SPRING_Y)
                spring_force_x = spring_force * math.cos(spring_angle_radians)
                spring_force_y = spring_force * math.sin(spring_angle_radians)

                x_velocity += spring_force_x / FRAMES_PER_SECOND
                y_velocity += spring_force_y / FRAMES_PER_SECOND

            global Clicked, ClickLocationX, ClickLocationY
            if Clicked:
                click_distance = euclidean_distance(ball_x, ball_y, ClickLocationX, ClickLocationY)
                click_force = ((CLICK_SPRING_CONSTANT * click_distance) / ball_mass)
                click_angle_radians = angle_horizon(ClickLocationX, ClickLocationY, ball_x, ball_y)
                click_force_x = click_force * math.cos(click_angle_radians)
                click_force_y = click_force * math.sin(click_angle_radians)

                x_velocity -= click_force_x / FRAMES_PER_SECOND
                y_velocity -= click_force_y / FRAMES_PER_SECOND

            # Apply friction
            if AIR_FRICTION_ENABLED:
                x_velocity -= x_velocity * ROLLING_FRICTION_COEFFICIENT_PER_SECOND
                y_velocity -= y_velocity * ROLLING_FRICTION_COEFFICIENT_PER_SECOND

            BALLS[index][1] = x_velocity
            BALLS[index][2] = y_velocity

        collision_time = 0.0

        # While there are collisions to resolve, resolve them 1 by 1 by:
        # 1) Find time of first collision t1
        # 2) Move all balls for time t1
        # 3) Update velocities for the balls that just collided
        while collision_time < FRAMES_PER_SECOND_INVERTED:
            earliest_collision_time = float('inf')
            earliest_collision_indices = [-1, -1]
            earliest_collision_type = "None"

            # 1) Find time of first collision
            for index, (ball, x_velocity, y_velocity, radius) in enumerate(BALLS):
                ball_x, ball_y = ball_center(canvas.coords(ball))

                # Check for wall collision
                delta_x = x_velocity * (FRAMES_PER_SECOND_INVERTED - collision_time)
                delta_y = y_velocity * (FRAMES_PER_SECOND_INVERTED - collision_time)

                proposed_ball_x = ball_x + delta_x
                proposed_ball_y = ball_y + delta_y

                if MIN_X + radius > proposed_ball_x:
                    time_of_collision = delta_x / x_velocity
                    if time_of_collision < earliest_collision_time:
                        earliest_collision_time = time_of_collision
                        earliest_collision_indices = [index, -1]
                        earliest_collision_type = "BOUNDARY_X"
                elif MAX_X - radius < proposed_ball_x:
                    time_of_collision = delta_x / x_velocity
                    if time_of_collision < earliest_collision_time:
                        earliest_collision_time = time_of_collision
                        earliest_collision_indices = [index, -1]
                        earliest_collision_type = "BOUNDARY_X"
                if MIN_Y + radius > proposed_ball_y:
                    time_of_collision = delta_y / y_velocity
                    if time_of_collision < earliest_collision_time:
                        earliest_collision_time = time_of_collision
                        earliest_collision_indices = [index, -1]
                        earliest_collision_type = "BOUNDARY_Y"
                elif MAX_Y - radius < proposed_ball_y:
                    time_of_collision = delta_y / y_velocity
                    if time_of_collision < earliest_collision_time:
                        earliest_collision_time = time_of_collision
                        earliest_collision_indices = [index, -1]
                        earliest_collision_type = "BOUNDARY_Y"

                # Check for ball to ball collision
                for other_index, (other_ball, other_x_velocity, other_y_velocity, other_radius) in enumerate(BALLS):
                    if index < other_index:
                        other_ball_x, other_ball_y = ball_center(canvas.coords(other_ball))
                        combined_radii = radius + other_radius

                        p = (ball_x, ball_y)
                        d = ((x_velocity - other_x_velocity) * FRAMES_PER_SECOND_INVERTED, (y_velocity - other_y_velocity) * FRAMES_PER_SECOND_INVERTED)
                        q = (other_ball_x, other_ball_y)
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
                                    earliest_collision_indices = (index, other_index)
                                    earliest_collision_type = "BALL"
                                    print("Found collision {0} and {1} at time {2}".format(index, other_index, earliest_collision_time))

            # If no collision, run until end of frame
            if earliest_collision_type == "None":
                earliest_collision_time = FRAMES_PER_SECOND_INVERTED - collision_time
            # there is a collision, run until that collision
            else:
                earliest_collision_time *= FRAMES_PER_SECOND_INVERTED - collision_time
            collision_time += earliest_collision_time

            # 2) Move all balls for time t1
            for index, (ball, x_velocity, y_velocity, radius) in enumerate(BALLS):
                move_ball(canvas, ball, x_velocity * earliest_collision_time, y_velocity * earliest_collision_time)
                BALLS[index][1] = x_velocity
                BALLS[index][2] = y_velocity

            index = earliest_collision_indices[0]
            other_index = earliest_collision_indices[1]

            # 3) Update velocities for the balls that just collided
            if earliest_collision_type == "BALL":
                ball, x_velocity, y_velocity, radius = BALLS[index]
                ball_x, ball_y = ball_center(canvas.coords(ball))
                ball_mass = math.pi * radius * radius
                other_ball, other_x_velocity, other_y_velocity, other_radius = BALLS[other_index]
                other_ball_x, other_ball_y = ball_center(canvas.coords(other_ball))
                other_ball_mass = math.pi * other_radius * other_radius

                tangent_vector_x = ball_y - other_ball_y
                tangent_vector_y = -(ball_x - other_ball_x)
                tangent_vector_magnitude = euclidean_distance(0, 0, tangent_vector_x, tangent_vector_y)
                tangent_vector_normalized = [tangent_vector_x / tangent_vector_magnitude, tangent_vector_y / tangent_vector_magnitude]
                relative_velocity = [x_velocity - other_x_velocity, y_velocity - other_y_velocity]

                dot_product = sum([i*j for (i, j) in zip(tangent_vector_normalized, relative_velocity)])
                velocity_component_on_tangent = [component * dot_product for component in tangent_vector_normalized]
                velocity_component_perpendicular_to_tangent = [i-j for (i, j) in zip(relative_velocity, velocity_component_on_tangent)]

                # weight_ratio = ball_mass / other_ball_mass
                # weight_ratio_inverted = 1.0 / (weight_ratio * FRAMES_PER_SECOND)

                BALLS[index][1] -= velocity_component_perpendicular_to_tangent[0]
                BALLS[index][2] -= velocity_component_perpendicular_to_tangent[1]

                print("\tx velocity -= {0}".format(velocity_component_perpendicular_to_tangent[0]))
                print("\ty velocity -= {0}".format(velocity_component_perpendicular_to_tangent[1]))

                BALLS[other_index][1] += velocity_component_perpendicular_to_tangent[0]
                BALLS[other_index][2] += velocity_component_perpendicular_to_tangent[1]

                print("\tother x velocity += {0}".format(velocity_component_perpendicular_to_tangent[0]))
                print("\tother y velocity += {0}".format(velocity_component_perpendicular_to_tangent[1]))

            elif earliest_collision_type == "BOUNDARY_X":
                BALLS[index][1] *= -0.8
            elif earliest_collision_type == "BOUNDARY_Y":
                BALLS[index][2] *= -0.8

            canvas.update()

        canvas.update()

        current_time = time.time_ns()
        while current_time < next_tick:
            time.sleep(SLEEP_TIME_S)
            current_time = time.time_ns()
        next_tick += UPDATE_RATE_NS

        frames += 1

        if current_time >= next_second:
            canvas.itemconfig(fps_text, text="FPS: {0}".format(frames))
            print("FPS: {0}".format(frames))
            next_second += SECOND_IN_NS
            frames = 0

    root.destroy()
    return 0


def move_ball(canvas, ball, x_velocity, y_velocity):
    canvas.move(ball, x_velocity, -y_velocity)


def ball_center(my_tuple):
    x_left, y_up, x_right, y_down = my_tuple
    return (x_left + x_right) / 2, MAX_Y - ((y_up + y_down) / 2)


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt(pow(x2-x1, 2) + pow(y2-y1, 2))


def angle_horizon(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


if __name__ == "__main__":
    main()
