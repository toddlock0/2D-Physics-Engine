import QuadTree
import random
import tkinter

WIDTH = 500
HEIGHT = 500
FRAMES_PER_SECOND_INVERTED = 1.0

# globals
run = True

# init tk
root = tkinter.Tk()

# create canvas
canvas = tkinter.Canvas(root, bg="white", height=int(HEIGHT), width=int(WIDTH))


def handler():
    global run
    run = False


root.title("Quad Tree Test")
root.protocol("WM_DELETE_WINDOW", handler)

canvas.pack()


def create_oval(canvas_m, quadtree_m, index, x, y, radius, x_velocity, y_velocity, add_to_quadtree):
    canvas_y = HEIGHT - y
    point = QuadTree.Point(index, x, y, x_velocity, y_velocity, radius)
    if add_to_quadtree:
        quadtree_m.insert(point)
    color = "red" if add_to_quadtree else "blue"
    canvas_m.create_oval(x - radius, canvas_y - radius, x + radius, canvas_y + radius, fill=color)
    return point


rectangle = QuadTree.Boundary(0, 0, WIDTH, HEIGHT)
quadtree = QuadTree.QuadTree(rectangle, 4)
create_oval(canvas, quadtree, 1, 100.0, 100.0, 50.0, 0.0, 0.0, True)

point_1 = create_oval(canvas, quadtree, 2, 201.0, 100.0, 50.0, -10.0, 0.01, False)
collision_time, collision_index = quadtree.check_collision(point_1, FRAMES_PER_SECOND_INVERTED)
assert collision_index == 1
print("A Collision found with {0} at time {1}".format(collision_index, collision_time))

point_2 = create_oval(canvas, quadtree, 3, 400.0, 400.0, 50.0, 0.02, 0.02, False)
collision_time, collision_index = quadtree.check_collision(point_2, FRAMES_PER_SECOND_INVERTED)
assert collision_index is None
print("B Collision found with {0} at time {1}".format(collision_index, collision_time))

while run:
    canvas.update()
