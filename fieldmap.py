#!/usr/bin/python3

# Create the field maps used by the simulation
# Implement the 2021 competition map and the ones for the Skills Challenge

# resources:
# https://firstfrc.blob.core.windows.net/frc2020/Manual/Sections/Section03.pdf
# https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf

# Work in inches since the manual and drawings are in inches

import sys
import logging
import argparse
import math
import matplotlib.pyplot as plt
import csv

# unless otherwise noted, these values are from the Manual

# all the tape is 2 inches
# draw the lines in the middle of the tape
tape_width = 2.0

field_width = 26.0 * 12 + 11.25      # 8.21 m
field_length = 52.0 * 12 + 5.25      # 15.98 m
initiate_line = 10 * 12.0
ball_rad = 3.5

trench_h = 55.5                 # from drawing
trench_w = 18.0 * 12.0

# target zone
target_h = 4 * 12.0
target_w = 30.0                 # from drawing
target_yc = 94.66               # from drawing

# trench balls
trench_ball_space = 36.0
trench_ball_yline = 27.75                           # from drawing
trench_pair_x = initiate_line + 130.36              # from drawing
trench_pair_y2 = field_width - target_yc - 191.43   # from drawing
trench_pair_y1 = trench_pair_y2 - 18.5              # in manual, section 3.5

# generator numbers
generator_truss = 12.0                 # 1 ft truss
generator_angle = math.radians(22.5)
# work with the center of the pillars, not the pads (do not really care about those)
generator_length = 13.0 * 12 + 1.5 - generator_truss
generator_width = 14.0 * 12 + 0.75 - generator_truss
generator_bar = 3                                 # 3 inch wide bars on the floor
generator_ball_space = 16.5                       # drawing 3-10 in manual

# 2 outside corners of the support pads, from drawing
generator_points = ((field_length/2 - 116.0, field_width/2 + 43.75),
                    (field_length/2 - 51.06, field_width/2 - 112.88))
# location of the balls, from drawing
# x is offset from initiate line, y is offset from goal
generator_balls = ((130.25, 19.79), (114.94, 26.13), (107.83, 50.54), (114.17, 65.84), (120.51, 81.14))

# ---------------
sc_field_length = 30 * 12
sc_field_width = 15 * 12


# return the x coordinate given the location index 1 - 11
def sc_x_pos(index):
    return index * 30


# return the y coordinate given the location index 'A' through 'E'
# note: you can use "upper()" method to guarantee it is upper case
def sc_y_pos(index):
    if index == "A":
        v = 150
    elif index == "B":
        v = 120
    elif index == "C":
        v = 90
    elif index == "D":
        v = 60
    elif index == "E":
        v = 30
    return v


def draw_circle(x, y, color="orange", radius=2.5, zorder=100):
    # zorder pulls the balls to the front, so they are on top of the lines
    c = plt.Circle((x, y), radius, fill=True, color=color, zorder=zorder)
    plt.gcf().gca().add_artist(c)
    return


def draw_marker(x, y, color="orange"):
    logging.info(f'Marker: ({x:.3f}, {y:.3f})')
    draw_circle(x, y, color=color)
    return


def draw_one_ball(x, y):
    logging.info(f'Ball: ({x:.3f}, {y:.3f})')
    draw_circle(x, y, radius=ball_rad, color='yellow')
    return


# there are always matching balls, reflected across the center
# so draw a pair
def draw_ball(x, y):
    logging.info(f'Ball: ({x:.3f}, {y:.3f})')
    draw_circle(x, y, radius=ball_rad, color='yellow')

    logging.info(f'Ball: ({field_length - x:.3f}, {field_width - y:.3f})')
    draw_circle(field_length - x, field_width - y, radius=ball_rad, color='yellow')
    return


def generator_to_field(x, y):
    '''Convert coordinates in "generator space" to actual field coordinates'''
    xf = x * math.cos(generator_angle) - y * math.sin(generator_angle) + field_length / 2
    yf = x * math.sin(generator_angle) + y * math.cos(generator_angle) + field_width / 2
    return xf, yf


def draw_truss(xg, yg):
    '''Given the center of the truss in Generator space, draw the box'''
    # logging.info(f'Draw truss: {xg}, {yg}')

    half = generator_truss / 2
    xcorners = (xg - half, xg - half, xg + half, xg + half, xg - half)
    ycorners = (yg - half, yg + half, yg + half, yg - half, yg - half)

    field_coord = [generator_to_field(r[0], r[1]) for r in zip(xcorners, ycorners)]
    plt.plot([r[0] for r in field_coord], [t[1] for t in field_coord], 'grey')
    return


# draw the Galactic Search map
# maybe add a parameter to indicate which ball path?
def draw_galactic_search(map_name):
    # set the plot area to be the size of the field
    axis1.set_xlim((0, sc_field_length))
    axis1.set_ylim((0, sc_field_width))

    # outer outline
    plt.plot((0, 0, sc_field_length, sc_field_length, 0), (0, sc_field_width, sc_field_width, 0, 0), 'green')

    # Draw the ball
    if map_name == "redA":
        draw_one_ball(sc_x_pos(6), sc_y_pos("A"))
        draw_one_ball(sc_x_pos(3), sc_y_pos("C"))
        draw_one_ball(sc_x_pos(5), sc_y_pos("D"))
    elif map_name == "redB":
        draw_one_ball(sc_x_pos(3), sc_y_pos("B"))
        draw_one_ball(sc_x_pos(7), sc_y_pos("B"))
        draw_one_ball(sc_x_pos(5), sc_y_pos("D"))
    elif map_name == "blueA":
        draw_one_ball(sc_x_pos(7), sc_y_pos("B"))
        draw_one_ball(sc_x_pos(9), sc_y_pos("C"))
        draw_one_ball(sc_x_pos(6), sc_y_pos("E"))
    elif map_name == "blueB":
        draw_one_ball(sc_x_pos(8), sc_y_pos("B"))
        draw_one_ball(sc_x_pos(6), sc_y_pos("D"))
        draw_one_ball(sc_x_pos(10), sc_y_pos("D"))
    return


def draw_slalom():
    # set the plot area to be the size of the field
    axis1.set_xlim((0, sc_field_length))
    axis1.set_ylim((0, sc_field_width))

    # outer outline
    plt.plot((0, 0, sc_field_length, sc_field_length, 0), (0, sc_field_width, sc_field_width, 0, 0), 'green')

    # Draw the ball
    draw_marker(sc_x_pos(1), sc_y_pos("B"))
    draw_marker(sc_x_pos(2), sc_y_pos("B"))
    draw_marker(sc_x_pos(1), sc_y_pos("D"))
    draw_marker(sc_x_pos(2), sc_y_pos("D"))
    draw_marker(sc_x_pos(4), sc_y_pos("D"))
    draw_marker(sc_x_pos(5), sc_y_pos("D"))
    draw_marker(sc_x_pos(6), sc_y_pos("D"))
    draw_marker(sc_x_pos(7), sc_y_pos("D"))
    draw_marker(sc_x_pos(8), sc_y_pos("D"))
    draw_marker(sc_x_pos(10), sc_y_pos("D"))
    return


def draw_bounce():
    # set the plot area to be the size of the field
    axis1.set_xlim((0, sc_field_length))
    axis1.set_ylim((0, sc_field_width))

    # outer outline
    plt.plot((0, 0, sc_field_length, sc_field_length, 0), (0, sc_field_width, sc_field_width, 0, 0), 'green')

    # Draw the marker
    draw_marker(sc_x_pos(1), sc_y_pos("B"))
    draw_marker(sc_x_pos(2), sc_y_pos("B"))
    draw_marker(sc_x_pos(4), sc_y_pos("B"))
    draw_marker(sc_x_pos(5), sc_y_pos("B"))
    draw_marker(sc_x_pos(7), sc_y_pos("B"))
    draw_marker(sc_x_pos(8), sc_y_pos("B"))
    draw_marker(sc_x_pos(10), sc_y_pos("B"))
    draw_marker(sc_x_pos(11), sc_y_pos("B"))
    draw_marker(sc_x_pos(1), sc_y_pos("D"))
    draw_marker(sc_x_pos(2), sc_y_pos("D"))
    draw_marker(sc_x_pos(3), sc_y_pos("D"))
    draw_marker(sc_x_pos(5), sc_y_pos("D"))
    draw_marker(sc_x_pos(7), sc_y_pos("D"))
    draw_marker(sc_x_pos(8), sc_y_pos("D"))
    draw_marker(sc_x_pos(10), sc_y_pos("D"))
    draw_marker(sc_x_pos(11), sc_y_pos("D"))
    draw_marker(sc_x_pos(3), sc_y_pos("E"))
    draw_marker(sc_x_pos(3), sc_y_pos("A"), "green")
    draw_marker(sc_x_pos(6), sc_y_pos("A"), "green")
    draw_marker(sc_x_pos(9), sc_y_pos("A"), "green")
    return


def draw_lightspeed():
    # set the plot area to be the size of the field
    axis1.set_xlim((0, sc_field_length))
    axis1.set_ylim((0, sc_field_width))

    # outer outline
    plt.plot((0, 0, sc_field_length, sc_field_length, 0), (0, sc_field_width, sc_field_width, 0, 0), 'green')

    # Draw the marker
    draw_marker(sc_x_pos(1), sc_y_pos("B"))
    draw_marker(sc_x_pos(3), sc_y_pos("B"))
    draw_marker(sc_x_pos(4), sc_y_pos("B"))
    draw_marker(sc_x_pos(6), sc_y_pos("B"))
    draw_marker(sc_x_pos(7), sc_y_pos("B"))
    draw_marker(sc_x_pos(9), sc_y_pos("B"))
    draw_marker(sc_x_pos(11), sc_y_pos("B"))
    draw_marker(sc_x_pos(6), sc_y_pos("A"))
    draw_marker(sc_x_pos(9), sc_y_pos("C"))
    draw_marker(sc_x_pos(1), sc_y_pos("D"))
    draw_marker(sc_x_pos(3), sc_y_pos("D"))
    draw_marker(sc_x_pos(4), sc_y_pos("D"))
    draw_marker(sc_x_pos(6), sc_y_pos("D"))
    draw_marker(sc_x_pos(7), sc_y_pos("D"))
    draw_marker(sc_x_pos(8), sc_y_pos("D"))
    draw_marker(sc_x_pos(9), sc_y_pos("D"))
    draw_marker(sc_x_pos(10), sc_y_pos("D"))
    return


def draw_barrel():
    # set the plot area to be the size of the field
    axis1.set_xlim((0, sc_field_length))
    axis1.set_ylim((0, sc_field_width))

    # outer outline
    plt.plot((0, 0, sc_field_length, sc_field_length, 0), (0, sc_field_width, sc_field_width, 0, 0), 'green')

    # Draw the marker
    draw_marker(sc_x_pos(1), sc_y_pos("B"))
    draw_marker(sc_x_pos(2), sc_y_pos("B"))
    draw_marker(sc_x_pos(8), sc_y_pos("B"))
    draw_marker(sc_x_pos(1), sc_y_pos("D"))
    draw_marker(sc_x_pos(2), sc_y_pos("D"))
    draw_marker(sc_x_pos(5), sc_y_pos("D"))
    draw_marker(sc_x_pos(10), sc_y_pos("D"))
    return


def draw_competition_map():
    '''Create the map for 2021 competition field'''

    # set the plot area to be the size of the field
    axis1.set_xlim((0, field_length))
    axis1.set_ylim((0, field_width))

    # general outline. Ignore the angled driver stations for now
    # initiation lines are white on the fied, but use red/blue for better visibility?
    plt.plot((0, 0, field_length, field_length, 0), (0, field_width, field_width, 0, 0), 'green')
    plt.plot((initiate_line + tape_width/2, initiate_line + tape_width/2), (0, field_width), 'blue')
    plt.plot((field_length - initiate_line - tape_width/2, field_length - initiate_line - tape_width/2),
             (0, field_width), 'red')

    # target zone
    # ignore the tape width - kind of messy to do
    plt.plot((field_length, field_length - target_w, field_length),
             (target_yc - target_h/2, target_yc, target_yc + target_h/2), 'blue')
    plt.plot((0, target_w, 0),
             (field_width - target_yc - target_h/2, field_width - target_yc, field_width - target_yc + target_h/2),
             'red')

    # draw the trenches
    x = (field_length - trench_w) / 2
    x2 = x + trench_w
    # numbers are for outside edge of tape, so subtract 1/2 the width
    x += tape_width/2
    x2 -= tape_width/2
    y = trench_h - tape_width/2
    plt.plot((x, x, x2, x2), (0, y, y, 0), 'blue')
    plt.plot((x, x, x2, x2), (field_width, field_width - y, field_width - y, field_width), 'red')

    # trench balls, first the three in line, and then the pair at the spinner
    for i in range(3):
        draw_ball(field_length / 2.0 + trench_ball_space * i, trench_ball_yline)
    y = 19.054                      # from onshape
    x = 128.405 + initiate_line     # from onshape
    draw_ball(trench_pair_x, trench_pair_y1)
    draw_ball(trench_pair_x, trench_pair_y2)

    # draw the bars between the trusses. note the color changes
    # TODO: update for 2021
    xg = (-generator_width + generator_truss - generator_bar) / 2
    pts = [[xg, (-generator_length + generator_truss) / 2], [xg, 0]]
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'blue')
    pts[0][1] *= -1
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'red')

    xg *= -1
    pts = [[xg, (-generator_length + generator_truss) / 2], [xg, 0]]
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'blue')
    pts[0][1] *= -1
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'red')

    xg = (generator_width - generator_truss) / 2
    yg = (-generator_length + generator_truss - generator_bar) / 2
    pts = ((-xg, yg), (xg, yg))
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'blue')

    pts = ((-xg, -yg), (xg, -yg))
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'red')

    # black bar up the center
    xg = (-generator_width + generator_truss - generator_bar) / 2
    pts = ((xg, 0), (-xg, 0))
    fpts = [generator_to_field(*p) for p in pts]
    plt.plot([p[0] for p in fpts], [pp[1] for pp in fpts], 'gray')

    # draw the truss outlines. Note that the width/length are to the centers
    for ix in range(-1, 2, 2):
        for iy in range(-1, 2, 2):
            draw_truss(ix * generator_width / 2, iy * generator_length / 2)

    # balls on shield generator bars
    # TODO: update for 2021
    for point in generator_balls:
        draw_ball(field_length - initiate_line - point[0], target_yc + point[1])

    return


def plot_trajectory(trajfile):
    with open(trajfile) as f:
        incsv = csv.DictReader(f)
        x = []
        y = []
        for row in incsv:
            if int(row['IsWaypoint']):
                draw_circle(float(row['X']), float(row['Y']), color='blue', radius=1)
            else:
                x.append(float(row['X']))
                y.append(float(row['Y']))
    plt.plot(x, y, 'red')
    return


map_choices = ('competition', 'redA', 'redB', 'blueA', 'blueB', "slalom", "bounce", "barrel", "lightspeed")

parser = argparse.ArgumentParser(description='Output a PNG of a simple field map')
parser.add_argument('--map', '-m', required=True, choices=map_choices, help='Which map to produce')
parser.add_argument('--trajectory', '-t', help='Trajectory CSV')
parser.add_argument('--verbose', '-v', default=0, action='count', help='Verbose')

args = parser.parse_args()

logging.basicConfig(format='%(message)s')
logging.getLogger().setLevel(logging.WARNING - 10*args.verbose)

# make sure to set the x and y directions to be equal
fig1, axis1 = plt.subplots(figsize=(9.6, 7.2))
axis1.set_aspect('equal')

if args.map == 'competition':
    draw_competition_map()
elif args.map in ('redA', 'redB', 'blueA', 'blueB'):
    draw_galactic_search(args.map)
elif args.map == "slalom":
    draw_slalom()
elif args.map == "bounce":
    draw_bounce()
elif args.map == "barrel":
    draw_barrel()
elif args.map == "lightspeed":
    draw_lightspeed()
else:
    logging.error(f"Map '{args.map}' not implemented")
    sys.exit(10)

outname = f"fieldmap_{args.map}.png"
if args.trajectory:
    plot_trajectory(args.trajectory)
    outname = f"trajectory_{args.map}.png"

plt.axis('off')
plt.savefig(outname, bbox_inches='tight', pad_inches=0, dpi=200, transparent=True)
plt.show()
