import tkinter
import time
import math
import random

# 1 seconds in nanoseconds
SECOND_IN_NS = 1000000000
FRAMES_PER_SECOND = 30.0
# Update rate in nanoseconds
UPDATE_RATE_NS = SECOND_IN_NS / FRAMES_PER_SECOND
UPDATES_RATE_PER_SECOND = SECOND_IN_NS / UPDATE_RATE_NS
# Sleep for 1/10th the update rate
SLEEP_TIME_S = 1.0 / (UPDATE_RATE_NS * 10.0)

NUMBER_BALLS = 100

GRAVITY_ACCELERATION = -9.81
GRAVITY_ACCELERATION_PER_TICK = GRAVITY_ACCELERATION / UPDATES_RATE_PER_SECOND
BALL_MASS_KG = 500.0
CEILING_SPRING_CONSTANT = 100.0
GROUND_HEIGHT = 50.0
CLICK_SPRING_CONSTANT = 500.0
ROLLING_FRICTION_COEFFICIENT = 0.99
ROLLING_FRICTION_COEFFICIENT_PER_SECOND = ROLLING_FRICTION_COEFFICIENT / UPDATES_RATE_PER_SECOND

GRAVITY_ENABLED = True
SPRING_ENABLED = False

SPRING_X = 150
SPRING_Y = 300

MIN_X = 0.0
MAX_X = 1200.0
MIN_Y = 0.0
MAX_Y = 800.0

BALL_STARTS = [(random.randrange(int(MAX_X)), random.randrange(int(MAX_Y))) for i in range(NUMBER_BALLS)]
BALLS = []
BALL_RADIUS = 10
BALL_COLORS = ["white", "red", "green", "blue", "yellow", "orange", "purple", "black", "cyan", "magenta"]

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

    fps_text = canvas.create_text(50, 10, fill="darkblue", font="Times 11 italic bold", text="FPS:")

    for ball_x, ball_y in BALL_STARTS:
        color = BALL_COLORS[random.randrange(len(BALL_COLORS))]
        BALLS.append([canvas.create_oval(ball_x - BALL_RADIUS, MAX_Y - (ball_y - BALL_RADIUS),
                                         ball_x + BALL_RADIUS, MAX_Y - (ball_y + BALL_RADIUS), fill=color), 0.0, 0.0])

    # add to window and show
    canvas.pack()

    current_time = time.time_ns()
    next_tick = current_time + UPDATE_RATE_NS
    next_second = current_time + SECOND_IN_NS
    frames = 0

    while run:
        for index, (ball, x_velocity, y_velocity) in enumerate(BALLS):
            ball_x, ball_y = ball_center(canvas.coords(ball))

            # print("Ball {0} center x {1} y {2}".format(index, ball_x, ball_y))

            if GRAVITY_ENABLED:
                y_velocity += GRAVITY_ACCELERATION_PER_TICK
            if SPRING_ENABLED:
                spring_distance = euclidean_distance(ball_x, ball_y, SPRING_X, SPRING_Y)
                spring_acceleration = ((CEILING_SPRING_CONSTANT * spring_distance) / BALL_MASS_KG) / UPDATES_RATE_PER_SECOND
                spring_angle_radians = angle_horizon(ball_x, ball_y, SPRING_X, SPRING_Y)
                spring_acceleration_x = spring_acceleration * math.cos(spring_angle_radians)
                spring_acceleration_y = spring_acceleration * math.sin(spring_angle_radians)

                x_velocity += spring_acceleration_x
                y_velocity += spring_acceleration_y
            global Clicked, ClickLocationX, ClickLocationY
            if Clicked:
                click_distance = euclidean_distance(ball_x, ball_y, ClickLocationX, ClickLocationY)
                click_acceleration = ((CLICK_SPRING_CONSTANT * click_distance) / BALL_MASS_KG) / UPDATES_RATE_PER_SECOND
                click_angle_radians = angle_horizon(ClickLocationX, ClickLocationY, ball_x, ball_y)
                click_acceleration_x = click_acceleration * math.cos(click_angle_radians)
                click_acceleration_y = click_acceleration * math.sin(click_angle_radians)

                x_velocity -= click_acceleration_x
                y_velocity -= click_acceleration_y

            # Apply friction
            #x_velocity -= pow(x_velocity, 2) * ROLLING_FRICTION_COEFFICIENT
            #y_velocity -= pow(y_velocity, 2) * ROLLING_FRICTION_COEFFICIENT
            x_velocity -= x_velocity * ROLLING_FRICTION_COEFFICIENT_PER_SECOND
            y_velocity -= y_velocity * ROLLING_FRICTION_COEFFICIENT_PER_SECOND

            # check if ball will be outside boundaries
            # if crossing boundary, bounce by inverting and losing some velocity
            proposed_ball_y = ball_y + y_velocity
            proposed_ball_x = ball_x + x_velocity

            # ball collision checking           O(N^2)!!!!!!
            for other_index, (other_ball, other_x_velocity, other_y_velocity) in enumerate(BALLS):
                if other_index > index:
                    other_ball_x, other_ball_y = ball_center(canvas.coords(other_ball))
                    if abs(proposed_ball_x - other_ball_x) < (2.0 * BALL_RADIUS) or abs(proposed_ball_y - other_ball_y) < (2.0 * BALL_RADIUS):
                        if euclidean_distance(proposed_ball_x, proposed_ball_y, other_ball_x, other_ball_y) < (2.0 * BALL_RADIUS):
                            # print("Collision! Balls {0} and {1} collided at X {2} Y {3}, other X {4} Y {5}".format(index, other_index, proposed_ball_x, proposed_ball_y, other_ball_x, other_ball_y))

                            tangent_vector_x = ball_y - other_ball_y
                            tangent_vector_y = -(ball_x - other_ball_x)
                            tangent_vector_magnitude = euclidean_distance(0, 0, tangent_vector_x, tangent_vector_y)
                            tangent_vector_normalized = [tangent_vector_x / tangent_vector_magnitude, tangent_vector_y / tangent_vector_magnitude]
                            relative_velocity = [x_velocity - other_x_velocity, y_velocity - other_y_velocity]
                            # print("Tangent vector {0} {1}, magnitude {2}".format(tangent_vector_x, tangent_vector_y, tangent_vector_magnitude))

                            dot_product = sum([i*j for (i, j) in zip(tangent_vector_normalized, relative_velocity)])
                            velocity_component_on_tangent = [component * dot_product for component in tangent_vector_normalized]
                            velocity_component_perpendicular_to_tangent = [i-j for (i, j) in zip(relative_velocity, velocity_component_on_tangent)]

                            x_velocity -= velocity_component_perpendicular_to_tangent[0]
                            y_velocity -= velocity_component_perpendicular_to_tangent[1]

                            BALLS[other_index][1] += velocity_component_perpendicular_to_tangent[0]
                            BALLS[other_index][2] += velocity_component_perpendicular_to_tangent[1]

            proposed_ball_y = ball_y + y_velocity
            proposed_ball_x = ball_x + x_velocity

            if ball_x > MIN_X + BALL_RADIUS >= proposed_ball_x:
                # print("Ball {0} bounced x left! X {0} proposed {1}".format(index, ball_x, proposed_ball_x))
                x_velocity *= -0.8
            elif ball_x < MAX_X - BALL_RADIUS < proposed_ball_x:
                # print("Ball {0} bounced x right! X {0} proposed {1}".format(index, ball_x, proposed_ball_x))
                x_velocity *= -0.8
            if ball_y > MIN_Y + BALL_RADIUS > proposed_ball_y:
                # print("Ball {0} bounced y bottom! Y {0} proposed {1}".format(index, ball_y, proposed_ball_y))
                y_velocity *= -0.8
            elif ball_y < MAX_Y - BALL_RADIUS < proposed_ball_y:
                # print("Ball {0} bounced y top! Y {0} proposed {1}".format(index, ball_y, proposed_ball_y))
                y_velocity *= -0.8

            move_ball(canvas, ball, x_velocity, y_velocity)
            BALLS[index][1] = x_velocity
            BALLS[index][2] = y_velocity
        
        canvas.update()

        current_time = time.time_ns()
        while current_time < next_tick:
            time.sleep(SLEEP_TIME_S)
            current_time = time.time_ns()
        next_tick += UPDATE_RATE_NS

        frames += 1

        if current_time >= next_second:
            canvas.itemconfig(fps_text, text="FPS: {0}".format(frames))
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
