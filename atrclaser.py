#!/usr/bin/env python

"""
Edits:
Added repeat layer parameter for repeating each cut int times


Metchit Laser Engraver Exporter

-----------------------------------
Modified by Mayhem2408 (John Revil) (https://github.com/mayhem2408/laser-gcode-exporter-inkscape-plugin)
Originally by Turnkey Tyranny (https://github.com/TurnkeyTyranny/laser-gcode-exporter-inkscape-plugin)
Special Thanks to Lauri Niskanen for their massive effects
Designed to run on Ramps 1.4 + Modified Marlin firmware on a K40 CO2 Laser Cutter and Diode Laser Engravers.
Based on think|haus gcode inkscape extension
Based on a script by Nick Drobchenko from the CNC club

***

Copyright (C) 2017 John Revill, jrevill@optusnet.com.au
Parts are Copyright (C) 2009 Nick Drobchenko, nick@cnc-club.ru
based on gcode.py (C) 2007 hugomatic... 
based on addnodes.py (C) 2005,2007 Aaron Spike, aaron@ekips.org
based on dots.py (C) 2005 Aaron Spike, aaron@ekips.org
based on interp.py (C) 2005 Aaron Spike, aaron@ekips.org

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
"""

"""

"""

###
###        Gcode tools
###

import inkex, simplestyle, simplepath
import cubicsuperpath, simpletransform, bezmisc

import os
import math
import bezmisc
import re
import copy
import sys
import time
import lxml

from collections import deque
import subprocess
import simplestyle

import getopt
from io import BytesIO

# Image processing for rastering
import png
import base64
import random
import PIL

from PIL import Image

################################################################################
###
###        Constants
###
################################################################################

VERSION = "1.0.2"

STRAIGHT_TOLERANCE = 0.0001
STRAIGHT_DISTANCE_TOLERANCE = 0.0001

LASER_ON = ""
LASER_OFF = ""

HEADER_TEXT = ""
FOOTER_TEXT = ""

BIARC_STYLE = {
    'biarc0': simplestyle.formatStyle({'stroke': '#88f', 'fill': 'none', 'strokeWidth': '1'}),
    'biarc1': simplestyle.formatStyle({'stroke': '#8f8', 'fill': 'none', 'strokeWidth': '1'}),
    'line': simplestyle.formatStyle({'stroke': '#f88', 'fill': 'none', 'strokeWidth': '1'}),
    'area': simplestyle.formatStyle({'stroke': '#777', 'fill': 'none', 'strokeWidth': '0.1'}),
}

# Inkscape group tag
SVG_GROUP_TAG = inkex.addNS("g", "svg")
SVG_PATH_TAG = inkex.addNS('path', 'svg')
SVG_IMAGE_TAG = inkex.addNS('image', 'svg')
SVG_TEXT_TAG = inkex.addNS('text', 'svg')
SVG_LABEL_TAG = inkex.addNS("label", "inkscape")

GCODE_EXTENSION = ".txt"
raster_gcode = ""
options = {}
AREA_WIDTH = 297 # The size of the cutting area
AREA_HEIGHT = 210 # Assumed to be A4: Outside this area, intensity is set to <=100 
#INTENSITY_CORRECTION = 0.25 # Maximum correction of laser intensity for the largest coordinate values , also affects feedrate modification

################################################################################
###
###        Common functions
###
################################################################################


###
###        Just simple output function for better debugging
###

class Logger(object):
    first = True
    enabled = True

    def __init__(self):
        home = os.getenv("HOME") or os.getenv("USERPROFILE")
        self.logpath = os.path.join(home, "thlaser.log")

    def write(self, s):
        if (not self.enabled):
            return

        if self.first and os.path.isfile(self.logpath):
            os.remove(self.logpath)
        self.first = False

        f = open(self.logpath, "a")
        f.write(str(s) + "\n")
        f.close()


# The global logger object
logger = Logger()


###
###        Point (x,y) operations
###
## Pretty much what it sounds like: defines some arithmetic functions that can be applied to points.
class P:
    def __init__(self, x, y=None):
        if not y == None:
            self.x, self.y = float(x), float(y)
        else:
            self.x, self.y = float(x[0]), float(x[1])

    def __add__(self, other):
        return P(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return P(self.x - other.x, self.y - other.y)

    def __neg__(self):
        return P(-self.x, -self.y)

    def __mul__(self, other):
        if isinstance(other, P):
            return self.x * other.x + self.y * other.y
        return P(self.x * other, self.y * other)

    __rmul__ = __mul__

    def __div__(self, other):
        return P(self.x / other, self.y / other)

    def mag(self):
        return math.hypot(self.x, self.y)

    def unit(self):
        h = self.mag()
        if h:
            return self / h
        else:
            return P(0, 0)

    def dot(self, other):
        return self.x * other.x + self.y * other.y

    def rot(self, theta):
        c = math.cos(theta)
        s = math.sin(theta)
        return P(self.x * c - self.y * s, self.x * s + self.y * c)

    def angle(self):
        return math.atan2(self.y, self.x)

    def __repr__(self):
        return '%f,%f' % (self.x, self.y)

    def pr(self):
        return "%.2f,%.2f" % (self.x, self.y)

    def to_list(self):
        return [self.x, self.y]


###


###        Functions to operate with CubicSuperPath
###

def csp_at_t(sp1, sp2, t):
    bez = (sp1[1][:], sp1[2][:], sp2[0][:], sp2[1][:])
    return bezmisc.bezierpointatt(bez, t)


def cspbezsplit(sp1, sp2, t=0.5):
    s1, s2 = bezmisc.beziersplitatt((sp1[1], sp1[2], sp2[0], sp2[1]), t)
    return [[sp1[0][:], sp1[1][:], list(s1[1])], [list(s1[2]), list(s1[3]), list(s2[1])],
            [list(s2[2]), sp2[1][:], sp2[2][:]]]


def cspbezsplitatlength(sp1, sp2, l=0.5, tolerance=0.01):
    bez = (sp1[1][:], sp1[2][:], sp2[0][:], sp2[1][:])
    t = bezmisc.beziertatlength(bez, l, tolerance)
    return cspbezsplit(sp1, sp2, t)


def cspseglength(sp1, sp2, tolerance=0.001):
    bez = (sp1[1][:], sp1[2][:], sp2[0][:], sp2[1][:])
    return bezmisc.bezierlength(bez, tolerance)


def csplength(csp):
    total = 0
    lengths = []
    for sp in csp:
        for i in xrange(1, len(sp)):
            l = cspseglength(sp[i - 1], sp[i])
            lengths.append(l)
            total += l
    return lengths, total


###
###        Distance calculattion from point to arc
###

def between(c, x, y):
    return x - STRAIGHT_TOLERANCE <= c <= y + STRAIGHT_TOLERANCE or y - STRAIGHT_TOLERANCE <= c <= x + STRAIGHT_TOLERANCE


def distance_from_point_to_arc(p, arc):
    P0, P2, c, a = arc
    dist = None
    p = P(p)
    r = (P0 - c).mag()
    if r > 0:
        i = c + (p - c).unit() * r
        alpha = ((i - c).angle() - (P0 - c).angle())
        if a * alpha < 0:
            if alpha > 0:
                alpha = alpha - 2 * math.pi
            else:
                alpha = 2 * math.pi + alpha
        if between(alpha, 0, a) or min(abs(alpha), abs(alpha - a)) < STRAIGHT_TOLERANCE:
            return (p - i).mag(), [i.x, i.y]
        else:
            d1, d2 = (p - P0).mag(), (p - P2).mag()
            if d1 < d2:
                return (d1, [P0.x, P0.y])
            else:
                return (d2, [P2.x, P2.y])


def get_distance_from_csp_to_arc(sp1, sp2, arc1, arc2, tolerance=0.001):  # arc = [start,end,center,alpha]
    n, i = 10, 0
    d, d1, dl = (0, (0, 0)), (0, (0, 0)), 0
    while i < 1 or (abs(d1[0] - dl[0]) > tolerance and i < 2):
        i += 1
        dl = d1 * 1
        for j in range(n + 1):
            t = float(j) / n
            p = csp_at_t(sp1, sp2, t)
            d = min(distance_from_point_to_arc(p, arc1), distance_from_point_to_arc(p, arc2))
            d1 = max(d1, d)
        n = n * 2
    return d1[0]


################################################################################
###        Biarc function
###        Calculates biarc approximation of cubic super path segment
###        splits segment if needed or approximates it with straight line
################################################################################

def biarc(sp1, sp2, z1, z2, depth=0, ):
    def biarc_split(sp1, sp2, z1, z2, depth):
        if depth < options.biarc_max_split_depth:
            sp1, sp2, sp3 = cspbezsplit(sp1, sp2)
            l1, l2 = cspseglength(sp1, sp2), cspseglength(sp2, sp3)
            if l1 + l2 == 0:
                zm = z1
            else:
                zm = z1 + (z2 - z1) * l1 / (l1 + l2)
            return biarc(sp1, sp2, depth + 1, z1, zm) + biarc(sp2, sp3, depth + 1, z1, zm)
        else:
            return [[sp1[1], 'line', 0, 0, sp2[1], [z1, z2]]]

    P0, P4 = P(sp1[1]), P(sp2[1])
    TS, TE, v = (P(sp1[2]) - P0), -(P(sp2[0]) - P4), P0 - P4
    tsa, tea, va = TS.angle(), TE.angle(), v.angle()
    if TE.mag() < STRAIGHT_DISTANCE_TOLERANCE and TS.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        # Both tangents are zerro - line straight
        return [[sp1[1], 'line', 0, 0, sp2[1], [z1, z2]]]
    if TE.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TE = -(TS + v).unit()
        r = TS.mag() / v.mag() * 2
    elif TS.mag() < STRAIGHT_DISTANCE_TOLERANCE:
        TS = -(TE + v).unit()
        inkex.errormsg("%s - %s - %s - %s" % (TE,TE.mag(),v, v.mag()))
        goo = v.mag()
        if goo == 0:
            goo=0.000001
        r = 1 / (TE.mag() / goo * 2)
        #r = 1 / (TE.mag() / v.mag() * 2)
        # ^ float division by zero
    else:
        r = TS.mag() / TE.mag()
    TS, TE = TS.unit(), TE.unit()
    tang_are_parallel = (
        (tsa - tea) % math.pi < STRAIGHT_TOLERANCE or math.pi - (tsa - tea) % math.pi < STRAIGHT_TOLERANCE)
    if (tang_are_parallel and
            ((
                                 v.mag() < STRAIGHT_DISTANCE_TOLERANCE or TE.mag() < STRAIGHT_DISTANCE_TOLERANCE or TS.mag() < STRAIGHT_DISTANCE_TOLERANCE) or
                         1 - abs(TS * v / (TS.mag() * v.mag())) < STRAIGHT_TOLERANCE)):
        # Both tangents are parallel and start and end are the same - line straight
        # or one of tangents still smaller then tollerance

        # Both tangents and v are parallel - line straight
        return [[sp1[1], 'line', 0, 0, sp2[1], [z1, z2]]]

    c, b, a = v * v, 2 * v * (r * TS + TE), 2 * r * (TS * TE - 1)
    if v.mag() == 0:
        return biarc_split(sp1, sp2, z1, z2, depth)
    asmall, bsmall, csmall = abs(a) < 10 ** -10, abs(b) < 10 ** -10, abs(c) < 10 ** -10
    if asmall and b != 0:
        beta = -c / b
    elif csmall and a != 0:
        beta = -b / a
    elif not asmall:
        discr = b * b - 4 * a * c
        if discr < 0:    raise ValueError, (a, b, c, discr)
        disq = discr ** .5
        beta1 = (-b - disq) / 2 / a
        beta2 = (-b + disq) / 2 / a
        if beta1 * beta2 > 0:    raise ValueError, (a, b, c, disq, beta1, beta2)
        beta = max(beta1, beta2)
    elif asmall and bsmall:
        return biarc_split(sp1, sp2, z1, z2, depth)
    alpha = beta * r
    ab = alpha + beta
    P1 = P0 + alpha * TS
    P3 = P4 - beta * TE
    P2 = (beta / ab) * P1 + (alpha / ab) * P3

    def calculate_arc_params(P0, P1, P2):
        D = (P0 + P2) / 2
        if (D - P1).mag() == 0: return None, None
        R = D - ((D - P0).mag() ** 2 / (D - P1).mag()) * (P1 - D).unit()
        p0a, p1a, p2a = (P0 - R).angle() % (2 * math.pi), (P1 - R).angle() % (2 * math.pi), (P2 - R).angle() % (
            2 * math.pi)
        alpha = (p2a - p0a) % (2 * math.pi)
        if (p0a < p2a and (p1a < p0a or p2a < p1a)) or (p2a < p1a < p0a):
            alpha = -2 * math.pi + alpha
        if abs(R.x) > 1000000 or abs(R.y) > 1000000 or (R - P0).mag < options.min_arc_radius:
            return None, None
        else:
            return R, alpha

    R1, a1 = calculate_arc_params(P0, P1, P2)
    R2, a2 = calculate_arc_params(P2, P3, P4)
    if R1 == None or R2 == None or (R1 - P0).mag() < STRAIGHT_TOLERANCE or (
                R2 - P2).mag() < STRAIGHT_TOLERANCE: return [[sp1[1], 'line', 0, 0, sp2[1], [z1, z2]]]

    d = get_distance_from_csp_to_arc(sp1, sp2, [P0, P2, R1, a1], [P2, P4, R2, a2])
    if d > options.biarc_tolerance and depth < options.biarc_max_split_depth:
        return biarc_split(sp1, sp2, z1, z2, depth)
    else:
        if R2.mag() * a2 == 0:
            zm = z2
        else:
            zm = z1 + (z2 - z1) * (R1.mag() * a1) / (R2.mag() * a2 + R1.mag() * a1)
        return [[sp1[1], 'arc', [R1.x, R1.y], a1, [P2.x, P2.y], [z1, zm]],
                [[P2.x, P2.y], 'arc', [R2.x, R2.y], a2, [P4.x, P4.y], [zm, z2]]]


################################################################################
###
###        Inkscape etc helper functions
###
################################################################################

# Returns true if the given node is a layer
def is_layer(node):
    return (node.tag == SVG_GROUP_TAG and
            node.get(inkex.addNS("groupmode", "inkscape")) == "layer")


def get_layers(document):
    layers = []
    root = document.getroot()
    for node in root.iterchildren():
        if (is_layer(node)):
            # Found an inkscape layer
            layers.append(node)
    return layers
    
def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def parse_layer_name(txt): # Get cutting style information from layername
    params = {}
    txt = txt.lower() # Make everything lower case for consistency
    txt = txt.replace(' ', '') # remove spaces
    for arg in txt.split(","):
        try:
            (field, value) = arg.split("=")
            #inkex.errormsg(field + ' is ' + value)
            if value.isdigit():
                value = int(value)
            elif is_number(value):
                value = float(value)
            # 
        except:
            # Flaggy args(Presence means true, otherwise false)
            #inkex.errormsg('Flaggy arg %s' % arg)
            value = True
            field = arg
            
        params[field] = value
        # logger.write("%s == %s" % (field, value))

    return params


################################################################################
###
###        Gcode tools class
###
################################################################################

class Gcode_tools(inkex.Effect):
    def __init__(self):
        inkex.Effect.__init__(self)

        outdir = os.getenv("HOME") or os.getenv("USERPROFILE")
        if (outdir):
            outdir = os.path.join(outdir, "Desktop")
        else: 
            outdir = os.getcwd()

        self.OptionParser.add_option("-d", "--directory", action="store", type="string", dest="directory",
                                     default=outdir, help="Directory for gcode file")
        self.OptionParser.add_option("-f", "--filename", action="store", type="string", dest="file", default="-1.0",
                                     help="File name")
        self.OptionParser.add_option("-x", "--Xoffset", action="store", type="float", dest="Xoffset", default="0.0",
                                     help="Offset along X")
        self.OptionParser.add_option("-y", "--Yoffset", action="store", type="float", dest="Yoffset", default="0.0",
                                     help="Offset along Y")
        self.OptionParser.add_option("", "--homing", action="store", type="int", dest="homing", default="4", help="")

        self.OptionParser.add_option("-m", "--Mfeed", action="store", type="int", dest="Mfeed", default="5000",
                                     help="Default Move Feed rate in unit/min")
        self.OptionParser.add_option("-p", "--feed", action="store", type="int", dest="feed", default="300",
                                     help="Default Cut Feed rate in unit/min")
        self.OptionParser.add_option("-l", "--laser", action="store", type="int", dest="laser", default="10",
                                     help="Default Laser intensity (0-100 %)")

        self.OptionParser.add_option("", "--line_type", action="store", type="string", dest="line_type", default="s",
                                     help="Default Line type (Solid/Pulsed)")
        self.OptionParser.add_option("", "--lpwmm", action="store", type="float", dest="lpwmm", default="2.54",
                                     help="Default Pulse Width (0.01-100)mm")
        self.OptionParser.add_option("", "--lplmm", action="store", type="float", dest="lplmm", default="1.5",
                                     help="Default Pulse Length (0.01-100)mm")

        self.OptionParser.add_option("-b", "--homebefore", action="store", type="inkbool", dest="homebefore",
                                     default=True, help="Home all before starting (G28)")
        self.OptionParser.add_option("-a", "--homeafter", action="store", type="inkbool", dest="homeafter",
                                     default=False, help="Home X Y at end of job")

        self.OptionParser.add_option("", "--biarc-tolerance", action="store", type="float", dest="biarc_tolerance",
                                     default="1", help="Tolerance used when calculating biarc interpolation.")
        self.OptionParser.add_option("", "--biarc-max-split-depth", action="store", type="int",
                                     dest="biarc_max_split_depth", default="4",
                                     help="Defines maximum depth of splitting while approximating using biarcs.")

        self.OptionParser.add_option("", "--unit", action="store", type="string", dest="unit",
                                     default="mm", help="Units")
        #self.OptionParser.add_option("", "--function", action="store", type="string", dest="function", default="Curve",help="What to do: Curve|Area|Area inkscape")

        self.OptionParser.add_option("", "--loft-distances", action="store", type="string", dest="loft_distances",
                                     default="10", help="Distances between paths.")
        self.OptionParser.add_option("", "--loft-direction", action="store", type="string", dest="loft_direction",
                                     default="crosswise", help="Direction of loft's interpolation.")
        self.OptionParser.add_option("", "--loft-interpolation-degree", action="store", type="float",
                                     dest="loft_interpolation_degree", default="2",
                                     help="Which interpolation use to loft the paths smooth interpolation or staright.")

        self.OptionParser.add_option("", "--min-arc-radius", action="store", type="float", dest="min_arc_radius",
                                     default="0.0005",
                                     help="All arc having radius less than minimum will be considered as straight line")
        self.OptionParser.add_option("", "--mainboard", action="store", type="string", dest="mainboard",
                                     default="marlin", help="Mainboard")
        self.OptionParser.add_option("", "--origin", action="store", type="string", dest="origin", default="topleft",
                                     help="Origin of the Y Axis")

        self.OptionParser.add_option("", "--bg_color", action="store", type="string", dest="bg_color",
                                     default="#ffffff", help="")
        self.OptionParser.add_option("", "--raster_method", action="store", type="string", dest="raster_method",
                                     default="base64", help="")
        self.OptionParser.add_option("", "--raster_direction", action="store", type="string", dest="raster_direction",
                                     default="h", help="")
        self.OptionParser.add_option("", "--burnwhite", action="store", type="inkbool", dest="burnwhite",
                                     default="True", help="")
        self.OptionParser.add_option("", "--raster_45deg_adjust", action="store", type="inkbool", dest="raster_45deg_adjust",
                                     default="True", help="")
        self.OptionParser.add_option("", "--raster_greyscale", action="store", type="string", dest="raster_greyscale",
                                     default="g", help="")
        self.OptionParser.add_option("", "--dl_power", action="store", type="int", dest="dl_power", default="100",
                                     help="")
        self.OptionParser.add_option("", "--resolution", action="store", type="int", dest="resolution", default="5",
                                     help="")

        self.OptionParser.add_option("", "--optires", action="store", type="inkbool", dest="optires", default="True",
                                     help="")
        self.OptionParser.add_option("", "--pulleyteeth", action="store", type="int", dest="pulleyteeth", default="20",
                                     help="")
        self.OptionParser.add_option("", "--pulleypitch", action="store", type="int", dest="pulleypitch", default="2",
                                     help="")
        self.OptionParser.add_option("", "--stepperrev", action="store", type="int", dest="stepperrev", default="200",
                                     help="")
        self.OptionParser.add_option("", "--steppermicro", action="store", type="int", dest="steppermicro", default="16",
                                     help="")

        self.OptionParser.add_option("", "--speed_ON", action="store", type="int", dest="speed_ON", default="1500",
                                     help="Rastering speed")
        self.OptionParser.add_option("", "--laser_min_value", action="store", type="int", dest="laser_min_value",
                                     default="6", help="")
        self.OptionParser.add_option("", "--laser_max_value", action="store", type="int", dest="laser_max_value",
                                     default="50", help="")
        self.OptionParser.add_option("", "--greyscale_depth", action="store", type="int", dest="greyscale_depth",
                                     default="256", help="")
        self.OptionParser.add_option("", "--white_cutoff", action="store", type="int", dest="white_cutoff",
                                     default="254", help="")
        self.OptionParser.add_option("", "--tab", action="store", type="string", dest="tab",
                                     default="tab", help="")
        self.OptionParser.add_option("", "--Xscale", action="store", type="int", dest="Xscale",
                                     default="1", help="")
        self.OptionParser.add_option("", "--Yscale", action="store", type="int", dest="Yscale",
                                     default="1", help="")
        self.OptionParser.add_option("", "--logging", action="store", type="inkbool", dest="logging",
                                     default="False", help="")                             
        #self.OptionParser.add_option("", "--flip_y", action="store", type="inkbool", dest="flip_y",
                                     #default="True", help="")   
        self.OptionParser.add_option("", "--m3commands", action="store", type="string", dest="m3commands",
                                     default="M3", help="")
        self.OptionParser.add_option("", "--dummylength", action="store", type="int", dest="dummylength",
                                     default="5", help="")
        self.OptionParser.add_option("", "--autodummy", action="store", type="inkbool", dest="autodummy",
                                     default="True", help="")
        self.OptionParser.add_option("", "--accel", action="store", type="int", dest="accel",
                                     default="500", help="")
        self.OptionParser.add_option("", "--feedratemod", action="store", type="float", dest="feedratemod",
                                     default="0", help="")
        self.OptionParser.add_option("", "--xscanline", action="store", type="int", dest="xscanline",
                                     default="1", help="")
        self.OptionParser.add_option("", "--gcodecomments", action="store", type="inkbool", dest="gcodecomments",
                                     default="True", help="")
        
    def parse_curve(self, path):
        xs, ys = 1.0, 1.0

        if (path['type'] == "vector"):
            lst = {}
            lst['type'] = "vector"
            lst['data'] = []
            for subpath in path['data']:
                lst['data'].append(
                    [[subpath[0][1][0] * xs, subpath[0][1][1] * ys], 'move', 0, 0]
                )
                for i in range(1, len(subpath)):
                    sp1 = [[subpath[i - 1][j][0] * xs, subpath[i - 1][j][1] * ys] for j in range(3)]
                    sp2 = [[subpath[i][j][0] * xs, subpath[i][j][1] * ys] for j in range(3)]
                    lst['data'] += biarc(sp1, sp2, 0, 0)

                lst['data'].append(
                    [[subpath[-1][1][0] * xs, subpath[-1][1][1] * ys], 'end', 0, 0]
                )
            return lst
        # Raster image data, cut/burn left to right, drop down a line, repeat in reverse until completed.
        else:
            # No need to modify
            return path
    # TODO: draw_curve is not used? Remove?
    def draw_curve(self, curve, group=None, style=BIARC_STYLE):
        if group == None:
            group = inkex.etree.SubElement(self.biarcGroup, SVG_GROUP_TAG)
        s, arcn = '', 0
        for si in curve:
            if s != '':
                if s[1] == 'line':
                    inkex.etree.SubElement(group, SVG_PATH_TAG,
                                           {
                                               'style': style['line'],
                                               'd': 'M %s,%s L %s,%s' % (s[0][0], s[0][1], si[0][0], si[0][1]),
                                               'comment': str(s)
                                           }
                                           )
                elif s[1] == 'arc':
                    arcn += 1
                    sp = s[0]
                    c = s[2]
                    a = ((P(si[0]) - P(c)).angle() - (P(s[0]) - P(c)).angle()) % (2 * math.pi)  # s[3]
                    if s[3] * a < 0:
                        if a > 0:
                            a = a - 2 * math.pi
                        else:
                            a = 2 * math.pi + a
                    r = math.sqrt((sp[0] - c[0]) ** 2 + (sp[1] - c[1]) ** 2)
                    a_st = (math.atan2(sp[0] - c[0], - (sp[1] - c[1])) - math.pi / 2) % (math.pi * 2)
                    if a > 0:
                        a_end = a_st + a
                    else:
                        a_end = a_st * 1
                        a_st = a_st + a
                    inkex.etree.SubElement(group, inkex.addNS('path', 'svg'),
                                           {
                                               'style': style['biarc%s' % (arcn % 2)],
                                               inkex.addNS('cx', 'sodipodi'): str(c[0]),
                                               inkex.addNS('cy', 'sodipodi'): str(c[1]),
                                               inkex.addNS('rx', 'sodipodi'): str(r),
                                               inkex.addNS('ry', 'sodipodi'): str(r),
                                               inkex.addNS('start', 'sodipodi'): str(a_st),
                                               inkex.addNS('end', 'sodipodi'): str(a_end),
                                               inkex.addNS('open', 'sodipodi'): 'true',
                                               inkex.addNS('type', 'sodipodi'): 'arc',
                                               'comment': str(s)
                                           })
            s = si

    def check_dir(self):
        if (os.path.isdir(self.options.directory)):
            if (os.path.isfile(self.options.directory + '/header')):
                f = open(self.options.directory + '/header', 'r')
                self.header = f.read()
                f.close()
            else:
                self.header = HEADER_TEXT
            if (os.path.isfile(self.options.directory + '/footer')):
                f = open(self.options.directory + '/footer', 'r')
                self.footer = f.read()
                f.close()
            else:
                self.footer = FOOTER_TEXT
        else:
            inkex.errormsg(("Directory specified for output gcode does not exist! Please create it."))
            return False

        return True
        
    # Correct laser intensity based on the distance the laser beam has to travel to reach the specified x,y coordinates. This sets a cap on the largest available/sensible intensity value in inkscape to 100/(1+INTENSITY_CORRECTION) = default 76.9 @ 0.3 , because values bigger than that would result in intensity values greater than 100 near the lower right corner of the cutting area.
   
    #def intens(self,x1,y1,x2,y2,i):
        ##x = max(x1,x2)
        ##y = max(-y1,-y2)
        ## TODO: If we ever start using svg user units, remove these calculations:
        #x = self.unitScale * (max(x1,x2) * self.options.Xscale + self.options.Xoffset)
        #y = self.unitScale * (min(y1,y2) * -self.options.Yscale + self.options.Yoffset)
        #out = i * (1.0 + (INTENSITY_CORRECTION * ((x+y)/(AREA_HEIGHT+AREA_WIDTH))))
        ##inkex.errormsg("Corrected intensity to %s based on coords: %s,%s" % ("{0:.1f}".format(out),"{0:.1f}".format(x),"{0:.1f}".format(y)))
        #if out > 100: out = 100
        #return out #"{0:.1f}".format(out)
        
    # Reduce feedrate to compensate for lost laser intensity 
    def feedratemod(self,x1,y1,x2,y2,i):
        INTENSITY_CORRECTION = self.options.feedratemod
        # x = self.unitScale * (max(x1,x2) * self.options.Xscale + self.options.Xoffset)
        # y = self.unitScale * (min(y1,y2) * -self.options.Yscale + self.options.Yoffset)
        x = self.unitScale * (((x1+x2)/2) * self.options.Xscale + self.options.Xoffset)
        y = self.unitScale * (((y1+y2)/2) * -self.options.Yscale + self.options.Yoffset)
        out = i * (1.0 - (INTENSITY_CORRECTION * ((x+y)/(AREA_HEIGHT+AREA_WIDTH))))
        #inkex.errormsg("Corrected intensity to %s based on coords: %s,%s" % ("{0:.1f}".format(out),"{0:.1f}".format(x),"{0:.1f}".format(y)))
        if out < 50: out = 50
        # Return modified feedrate value
        return out #"{0:.1f}".format(out)


    # Turns a list of arguments into gcode-style parameters (eg (1, 2, 3) -> "X1 Y2 Z3"),
    # taking scaling, offsets and the "parametric curve" setting into account
    def make_args(self, c):
        c = [c[i] if i < len(c) else None for i in range(6)]
        if c[5] == 0:
            c[5] = None

        s = ["X", "Y", "Z", "I", "J", "K"]
        s1 = ["", "", "", "", "", ""]

        m = [self.options.Xscale, -self.options.Yscale, 1,
             self.options.Xscale, -self.options.Yscale, 1]
        a = [self.options.Xoffset, self.options.Yoffset, 0, 0, 0, 0]
        # self.unitScale * (x * self.options.Xscale + self.options.Xoffset)
        # self.unitScale * (y * -self.options.Yscale + self.options.Yoffset)
        if (self.options.origin != 'topleft'):
            a[1] += self.pageHeight

        args = []
        for i in range(6):
            if c[i] != None:
                value = self.unitScale * (c[i] * m[i] + a[i])
                args.append(s[i] + ("%.3f" % value) + s1[i])
        return " ".join(args)
    

    # Make raster gcode that burns an area in greyscale. base64 makes file transfer efficient.
    def Rasterbase64(self, id, rasterspeed=None, resolution=None, max_power=None, min_power=None, raster_dir=None, raster_greyscale=None):

        def gccomment(gc):
            if (self.options.gcodecomments): # remove gcode comments
                enablecomments = False
            else:
                enablecomments = True

            sgc = '\n'
            if (gc == ''):
                sgc = sgc 
            else:
                if (enablecomments):
                    sgc = ' ' + gccommentstart + gc + gccommentend + sgc
            return sgc

        raster_gcode = ''
        if rasterspeed==None:
            rasterspeed = self.options.rasterspeed
        if resolution==None or resolution <= 0:
            resolution = self.options.resolution
        if max_power==None or max_power <= 0:
            max_power = self.options.laser_max_value
        if min_power==None or min_power < 0:
            min_power = self.options.laser_min_value
        if raster_dir==None:
            raster_dir = self.options.raster_direction
        raster_dir = str(raster_dir)
        if raster_greyscale==None:
            raster_greyscale = self.options.raster_greyscale

        if (self.options.raster_greyscale == 'l'):
            gs_depth = int(2)
        else:
            gs_depth = int(self.options.greyscale_depth)
        F_G01 = rasterspeed
        F_G00 = self.options.Mfeed

        scale = resolution

        if (raster_dir == '45'):
            xscanline = 1
        else:
            xscanline = self.options.xscanline

        lineresolution = resolution
        pixelresolution = resolution * xscanline
		
        DPI = pixelresolution * 25.4 # resolution is actually pixels per mm
        
        # Get dpi from resolution
        # resolution : default 11 : cuts per mm, size of pixel
        # Therefore size of pixel = 1/11 = 0.0909 mm
        pixelsize = "{0:.5f}".format(1 / (float(pixelresolution)))
        if (raster_dir == '45'):
            pixelsize = float(pixelsize) * math.sqrt(2)
            pixelsize45 = pixelsize # float(pixelsize) * math.sqrt(2)

        if (self.options.raster_45deg_adjust == True) and (raster_dir == '45'):
            DPI = (lineresolution * 25.4) / math.sqrt(2)
            resolution = resolution / math.sqrt(2)
            lineresolution = lineresolution / math.sqrt(2)
            pixelresolution = lineresolution / math.sqrt(2)
            scale = lineresolution
            pixelsize = "{0:.5f}".format(1 / (float(DPI) / 25.4))
            raster_gcode += '; Resolution has been optimised for 45deg rastering\n'

        if (self.options.optires == True):
            # =1/(ROUND((1/J34)/(H10/H6),0)*H$11)
            # =1/(ROUND((1/UserResolution)/(DistancePerRev/StepsPerRev),0)*DistancePerStep)
            # TODO: Allow user to set a single Steps Per mm value instead
            # StepsPermm/(Round(StepsPermm/UserResolution,0))

            #inkex.errormsg("self.options.stepperrev: %s" % self.options.stepperrev)
            #inkex.errormsg("self.options.steppermicro: %s" % self.options.steppermicro)
            #inkex.errormsg("self.options.pulleyteeth: %s" % self.options.pulleyteeth)
            #inkex.errormsg("self.options.pulleypitch: %s" % self.options.pulleypitch)
            #inkex.errormsg("resolution: %s" % resolution)

            stepsperrev = float(self.options.stepperrev * self.options.steppermicro)
            distperrev = float(self.options.pulleyteeth * self.options.pulleypitch)
            distperstep = float(float(distperrev) / float(stepsperrev))

            #inkex.errormsg("stepsperrev: %s" % stepsperrev)
            #inkex.errormsg("distperrev: %s" % distperrev)
            #inkex.errormsg("distperstep: %s" % distperstep)

            pixelresolution = 1.0 / (round((1.0 / pixelresolution) / ((distperrev) / (stepsperrev)),0) * distperstep)
            lineresolution = 1.0 / (round((1.0 / lineresolution) / ((distperrev) / (stepsperrev)),0) * distperstep)
            scale = lineresolution
            DPI = pixelresolution * 25.4
            pixelsize = "{0:.5f}".format(1 / (float(pixelresolution)))
            raster_gcode += '; Resolution has been optimised for stepper motor steps\n'

        #This extension assumes that your copy of Inkscape is running at 90dpi (it is by default)
        #R = mm per pixel
        
        #R = 1 / dots per mm
        #90dpi = 1 / (90 / 25.4)
        #Rasters are exported internally at 270dpi. 
        #So R = 1 / (270 / 25.4) 
        # pixelsize = 1 / (DPI / 25.4)
        #     = 0.09406 raster_mm_per_pulse
        
        xscanline = pixelresolution / lineresolution
		
        raster_gcode += ';Line DPI: %s; That means %s lines per mm \n' %(lineresolution * 25.4, lineresolution)
        raster_gcode += ';Pixel DPI: %s; That means %s pixels per mm on each line\n' %(pixelresolution * 25.4, pixelresolution)
        raster_gcode += ';Pixel DPI is %sx Line DPI\n' %(xscanline)
        current_file = self.args[-1]
        # exported_png = self.getTmpPath() + "laser_temp.png"
        exported_png = os.path.join(self.options.directory, 'laser_temp.png')
        raster_gcode += '; Exported PNG file: ' + exported_png + "\n"
        command = "inkscape \"%s\" -i \"%s\" -j -b\"%s\" -C --export-png=\"%s\" -d %s" % (
            current_file, id, self.options.bg_color, exported_png, (pixelresolution * 25.4))
        #raster_gcode += '; CMD: ' + command + "\n"
        # command="inkscape -C -e \"%s\" -b\"%s\" %s -d %s" % (exported_png, bg_color, current_file, DPI)
        
        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return_code = p.wait()
        f = p.stdout
        err = p.stderr

        
        imgresize = PIL.Image.open(exported_png)
        if raster_dir == 'h':
            imgresize = imgresize.resize((imgresize.size[0], int(imgresize.size[1]/xscanline)), PIL.Image.ANTIALIAS)
        else:
            if raster_dir == 'v': 
                imgresize = imgresize.resize((int(imgresize.size[0]/xscanline), imgresize.size[1]), PIL.Image.ANTIALIAS)
            else:
                # TODO: Scanline resolution increaseing is not implemented in 45deg yet. Still working on the maths
                imgresize = imgresize.resize((int(imgresize.size[0]/xscanline), int(imgresize.size[1]/xscanline)), PIL.Image.ANTIALIAS)
        
        imgresize.save(exported_png)
        
        #inkex.errormsg(str(metadata))
        #def changerange(oldvalue,oldmin,oldmax,newmin,newmax):
            #oldrange = oldmax-oldman
            #newrange = newmax-newmin
            #return (((oldvalue-oldmin)*newrange)/oldrange)+newmin
        def changerange256(oldvalue,newmin,newmax):
            newrange = newmax-newmin
            newvalue = int(((oldvalue*newrange)/255)+newmin)
            if int(newvalue) <= int(newmin):
                newvalue = 0
            return int(newvalue)

        ######## rescale for from 0-255 to 175-255
        pngrescale = png.Reader(exported_png)
        w, h, pixels, metadata = pngrescale.read_flat()
        rescale_array = [[0 for i in range(w)] for j in range(h)]

        for y in range(h):
            for x in range(w):
                pos = (x + y * w) * 4 if metadata['alpha'] else (x + y * w) * 3
                # Convert to greyscale using a method that simulates human vision, and flip value around
                avg = (round(pixels[pos] * 0.21 + pixels[pos + 1] * 0.72 + pixels[pos + 2] * 0.07,0))
                # Change Gamma  encoded = ((original / 255) ^ (1 / gamma)) * 255
                #avg = math.pow((avg / 255),(1.0/1.5)) * 255 # Lightend midrange of image
                #avg = 255-(math.pow(((255-avg) / 255),(1.0/2)) * 255) # Negative Gamma (Darkens image)
                # change the pixel brightness range
                #avg = changerange256(avg,64,255) # with 0 being solid black, only go down to 175 which means everything will be dithered with a max of 31%

                iavg = 255.0-avg
                avg = (iavg / 2.55)
                oavg = avg # Original avg for comparison
                #if avg <= 0.0:
                if avg <= 1.0:
                    avg = 0
                else:
                    if avg > 100.0:
                        avg = 100

                mod = 0.9 # Affect how close to the quadratic curve the final value should go: 1=full reduction (max 63from linear) 0= linear (max 0 from linear)
                avg = int((mod*100.0*avg + (1-mod)*(avg**2))/100.0)

                rescale_array[y][x] = int(255-(avg*2.55))

        png.from_array(rescale_array,'L').save(exported_png)

        rgbimg = Image.open(exported_png)
        rgbimg = rgbimg.convert('RGB')
        rgbimg.save(exported_png, 'PNG' )
                
        
        def nearestcolour(pixcolour,levels):
            #newcolour = 0
            colourdiv = 255.0 / float(levels-1) # 255.0 / 8 = 31.875
            newcolour = int(int((float(pixcolour) / float(colourdiv))+0.5) * colourdiv)
            #inkex.errormsg('ret nc:%s' %(newcolour))
            return int(newcolour)

        ######## Dithering
        if (raster_greyscale == 'd'):
            # JJN Dithering
            err_coeffs = [0.0, 0.0, 0.0, 7.0, 5.0, 3.0, 5.0, 7.0, 5.0, 3.0, 1.0, 3.0, 5.0, 3.0, 1.0]
            er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/48.0, err_coeffs)
            # Floyd-Steinberg Dithering
            #err_coeffs = [0.0, 0.0, 0.0, 7.0, 0.0, 0.0, 3.0, 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/16.0, err_coeffs)
            # Floyd-Steinberg (Fake) Dithering
            #err_coeffs = [0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 3.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/8.0, err_coeffs)
            # Stucki Dithering
            #err_coeffs = [0.0, 0.0, 0.0, 8.0, 4.0, 2.0, 4.0, 8.0, 4.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0]
            #er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/42.0, err_coeffs)
            # Atkinson Dithering (MacPaint Days)
            #err_coeffs = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0]
            #er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/8.0, err_coeffs)
            # Burkes Dithering
            #err_coeffs = [0.0, 0.0, 0.0, 8.0, 4.0, 2.0, 4.0, 8.0, 4.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            #er1, er2, er3, er4, er5, er6, er7, er8, er9, er10, er11, er12, er13, er14, er15 = map(lambda x : float(x)/32.0, err_coeffs)
            for y in range(h):
                for x in range(w):

                    # dither using JJN algorithm
                    oldpixel = rescale_array[y][x]
                    newpixel = nearestcolour(oldpixel,2)

                    rescale_array[y][x] = newpixel
                    quant_error = float (oldpixel - newpixel)
                    # Same row
                    #if ( x > 1):
                        #rescale_array[y+1][x-2] += (er6 * quant_error)
                    #if ( x > 0):
                        #rescale_array[y+1][x-1] += (er7 * quant_error)
                    #rescale_array[y+1][x] += (er8 * quant_error)
                    if ( x < w - 1):
                        rescale_array[y][x+1] += (er4 * quant_error)
                    if ( x < w - 2):
                        rescale_array[y][x+2] += (er5 * quant_error)
                    # one row down
                    if ( x > 1) and (y < h - 1):
                        rescale_array[y+1][x-2] += (er6 * quant_error)
                    if ( x > 0) and (y < h - 1):
                        rescale_array[y+1][x-1] += (er7 * quant_error)
                    if (y < h - 1):
                        rescale_array[y+1][x] += (er8 * quant_error)
                    if ( x < w - 1) and (y < h - 1):
                        rescale_array[y+1][x+1] += (er9 * quant_error)
                    if ( x < w - 2) and (y < h - 1):
                        rescale_array[y+1][x+1] += (er10 * quant_error)
                    # two rows down
                    if ( x > 1) and (y < h - 2):
                        rescale_array[y+2][x-2] += (er11 * quant_error)
                    if ( x > 0) and (y < h - 2):
                        rescale_array[y+2][x-1] += (er12 * quant_error)
                    if (y < h - 2):
                        rescale_array[y+2][x] += (er13 * quant_error)
                    if ( x < w - 1) and (y < h - 2):
                        rescale_array[y+2][x+1] += (er14 * quant_error)
                    if ( x < w - 2) and (y < h - 2):
                        rescale_array[y+2][x+1] += (er15 * quant_error)
                    
                    
            png.from_array(rescale_array,'L').save(exported_png)
            ditheredimg = Image.open(exported_png)
            #ditheredimg = ditheredimg.convert('1')
            ditheredimg = ditheredimg.convert('RGB')
            ditheredimg.save(exported_png, 'PNG' )

        ######## Open the image that was exported with Inkscape
        reader = png.Reader(exported_png)
        w, h, pixels, metadata = reader.read_flat()
        
        ######## Make an array containing the image in greyscale
        # if direction is Horizontal or 45deg
        #if self.options.raster_direction != 'V':
        if raster_dir != 'v':
            grey_array = [[255 for i in range(w)] for j in range(h)]
            tmp_array = [[0 for i in range(w)] for j in range(h)]
            for y in range(h):
                if (self.options.origin == 'topleft'):
                    y_tmp = y
                else:
                    y_tmp = h-y-1

                for x in range(w):
                    pos = (x + y * w) * 4 if metadata['alpha'] else (x + y * w) * 3
                    # Convert to greyscale using a method that simulates human vision, and flip value around
                    avg = 255 -(round(pixels[pos] * 0.21 + pixels[pos + 1] * 0.72 + pixels[pos + 2] * 0.07,0))
                    # Reduce color depth
                    # reduced = int((int((avg/(float(256)/100)))) * (float(255) / (100 -1)))
                    if (gs_depth < 256): # No need to use this function is full 256 greys are being used. Doesn't make any difference to the result, but may speed up 256 greyscale
                        reduced = int(int(avg/(256.0/gs_depth))*(255.0/(gs_depth-1)))
                    else:
                        reduced = int(avg)

                    tmp_array[y_tmp][x] = reduced # Export a greyscale PNG based on this array
                    mod = 0.9 # Affect how close to the quadratic curve the final value should go: 1=full reduction (max 63from linear) 0= linear (max 0 from linear)
                    grey_array[y_tmp][x] = int((mod*255*reduced + (1-mod)*(reduced**2))/255)

        # if Vertical
        else:
            grey_array = [[255 for i in range(h)] for j in range(w)]
            tmp_array = [[0 for i in range(h)] for j in range(w)]
            for y in range(h):
                if (self.options.origin == 'topleft'):
                    y_tmp = y
                else:
                    y_tmp = h-y-1

                for x in range(w):
                    pos = (x + y * w) * 4 if metadata['alpha'] else (x + y * w) * 3
                    # Convert to greyscale using a method that simulates human vision, and flip value around
                    avg = 255 -(round(pixels[pos] * 0.21 + pixels[pos + 1] * 0.72 + pixels[pos + 2] * 0.07,0))
                    # Reduce color depth
                    if (gs_depth < 256):
                        reduced = int(int(avg/(256.0/gs_depth))*(255.0/(gs_depth-1)))
                    else:
                        reduced = avg

                    tmp_array[x][y_tmp] = reduced # Export a greyscale PNG based on this array
                    mod = 0.9 # Affect how close to the quadratic curve the final value should go: 1=full reduction (max 63from linear) 0= linear (max 0 from linear)
                    grey_array[x][y_tmp] = int((mod*255*reduced + (1-mod)*(reduced**2))/255)
        
        # Make preview png file
        png.from_array(tmp_array,'L').save(exported_png)

        ####### Make GCode from image data
        #if self.options.flip_y == False:
            #grey_array.reverse()

        Laser_ON = False

        F_G01 = rasterspeed
        F_G00 = self.options.Mfeed

        # HOMING
        if self.options.homing == 1:
            raster_gcode += 'G28; home all axes\n'
        elif self.options.homing == 2:
            raster_gcode += '$H; home all axes\n'
        elif self.options.homing == 3:
            #raster_gcode += 'G00 X0 Y0 ;Returning to origin\n'
            raster_gcode += ('G00 X0 Y0 F%s ;Returning to origin\n' % F_G00)
        else:
            pass

		########## Pre and Post line acceleration spacing by John Revill
        # Compacts Gcode by combining consecutive pixels with the same value into a single command.
        
        # Converts greyscale range into laser intensity values suitable for the user's laser machine.
        def intensity(pix):
            # return "{0:.1f}".format(max_power - (((max_power - self.options.laser_min_value) * pix) / float(255)))
            return "{0:.1f}".format((max_power  * pix) / float(255))
        
        # the rasterto function will let the S (power) value to the max_power of the layer or default. We want to keep it this way just for safely in the even
        # data gets messed up there is no way the laser will ever power more than the max_power
        # so the idea here is the S is 50, the data needs to represent 255 to get 50% power
        # The problem is the min_power. If it is set to 20%, once S50 hits that, it becomes 12-13%. So we need to set the lowest valuse proportionally higher
        # based the the max_power of S to get a real world range between 20-50.
        # ie. Max = 50, Min = 25
        # scale = 100 / 50 = 2
        # New Min will be 25 * scale = 25 * 2 = 50
        # Once S50 hits a 50% value, it will become 25% which is the min.
        def rescale(data_array,minp,maxp):
            new_scale = 100 / maxp
            new_min = minp * new_scale
            m = 0
            while m < len(data_array):
                data_array[m] = changerange256(data_array[m],int(new_min*2.55),255)
                m += 1
            return data_array


        def G0(x,y,speed="fast",usescale=True,comment=""):
            g ="" #"M649 S0 B0 D0\n"
          
            if speed == "fast":
                speed = F_G00
            else:
                speed = F_G01
            # Tiny bit of randomness added to reduce patterns, hopefully
            speed = "{0:.1f}".format(self.feedratemod(x,y,x,y,speed)+ random.randint(-5,3))
            #return g + 'G0 X' + "{0:.3f}".format((float(x) / scale)) + ' Y' + "{0:.3f}".format(float(y) / scale) + ' F' + speed + '; ' + comment + '\n'
            if (usescale):
                outstr = g + 'G0 X' + "{0:.3f}".format((float(x) / scale)) + ' Y' + "{0:.3f}".format(float(y) / scale) + ' F' + speed + gccomment(comment)
            else:
                outstr = g + 'G0 X' + "{0:.3f}".format(float(x)) + ' Y' + "{0:.3f}".format(float(y)) + ' F' + speed + gccomment(comment)
            return outstr
        
        def G1(x,y,i=""):
            if i:
                i=' S' + i
            return 'G1 X' + "{0:.3f}".format((float(x) / scale)) + ' Y' + "{0:.3f}".format(float(y) / scale) + ' F' + str(F_G01) + i +'\n'
            

        #def randomlength():
        #    return random.randint(35,51)
        def randomlength():
            # random length is designed to prevent patterns from appearing in the image due
            # consistant repetition.
            # base64 turns 3 characters into 4.
            # Base64 ABAA = Binary 000000 000001 000000 000000 
            # converts to 00000000 00010000 00000000 or 0x00, 0x10, 0x00
            # Base64 ABA= = Binary 000000 000001 000000 xxxxxx
            # converts to 00000000 00010000 00xxxxxx or 0x00, 0x10
            # Base64 AB== = Binary 000000 000001 xxxxxx xxxxxx
            # converts to 00000000 0001xxxx xxxxxxxx or 0x00

            # if you require 3 binary characters, you get 4 in return.
            # if you only reuire 2 binary characts, you still get 4 in return, but the last
            # character will be '=' representing 3-1 = 2 characters
            # if you only require 1 binary character, you must still return 4 characters, but the last
            # characters will be '==' representing 3-2 = 1 character.
            # for your purpose, the = signs are wasted bytes so we need to limit the length to 
            # multiples of 3 to revent the = signs and wasted bandwidth.
            chucksize = random.randint(13,17) * 3
            return chucksize
            
        def get_chunks(arr):
            # A bit of randomness added to get rid of noticeable patterns in rastered photographs
            chunk_size = randomlength()
            chunks  = [ arr[start:start+chunk_size] for start in range(0, len(arr), chunk_size)]
            # inkex.errormsg("Chunks: %s" % chunks)
            return chunks 
        
        def rasterto(from_px, to_px, line_px, data, di=1): # di = direction 1=right, 0=left, 3=vertical away from Origin, 2=vertical towards origin 
            # inkex.errormsg("Rasterto: %s-%s, %s, %s, %s" % (fromx, tox, y, data, len(data)))
            # Break down data on multiple lines if necessary
            output=""
            # pixel = 0.09406 # Height of a pixel / row in raster image //TODO: Make option and calculate the correct export dpi automatically
            first=True
            d = get_chunks(data)

            # Work around: Correct the Half delay/pixel shift that occurs in the Marlin firmware.
            if (di == 0) or (di == 2):
                from_px += 1
            else:
                from_px += 0


            if (di <=1): # Horizontal
                output += G0(from_px/pixelresolution,line_px/lineresolution,"slow",False,"Line n.%s" % y)
            else: # Vertical
                output += G0(line_px/lineresolution,from_px/pixelresolution,"slow",False,"Line n.%s" % y)
	    
            for dat in d:

                #b64 = base64.b64encode("".join(chr(c) for c in dat),"99")
                b64 = base64.b64encode("".join(chr(c) for c in dat),"+/")
                l = str(len(b64))
                if first:
                    #First line contains direction
                    output += 'G7 @' +str(di)+ ' L' +l+ ' D' + b64 + '\n'
                    first = False
                else:
                    output += 'G7 L' +l+ ' D' + b64 + '\n'

            #if di == 0 or di == 1:
                # output += G0(from_px/pixelresolution,line_px/lineresolution,"slow",False,"Raster data carrier") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
            #elif di == 2 or di == 3:
                #output += G0(xyline,topx,"slow", "Raster data carrier") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
                ###output += G1(line_px/lineresolution,from_px/pixelresolution,"0") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.

            return output

        def rastertoH(fromx, tox, y, data, di=1): # di = direction 1=right, 0=left
            #xscale = self.options.xscanline
            # inkex.errormsg("Rasterto: %s-%s, %s, %s, %s" % (fromx, tox, y, data, len(data)))
            # Break down data on multiple lines if necessary
            output=""
            # pixel = 0.09406 # Height of a pixel / row in raster image //TODO: Make option and calculate the correct export dpi automatically
            first=True
            d = get_chunks(data)

            # Work around: Correct the Half delay/pixel shift that occurs in the Marlin firmware.
            if di == 0:
                fromx += 1
            else:
                fromx += 0

            fromxs = fromx / xscale

            output += G0(fromxs,y,"slow",True,"Line n.%s" % y)
            #output += 'M649 S'+str(max_power)+' B2 D0 R'+str(pixelsize)+'\n'
            #pixelsizel = float(pixelsize) / xscale
            #output += 'M649 S'+str(max_power)+' B2 D0 R'+str(pixelsizel)+'\n'

            for dat in d:

                #b64 = base64.b64encode("".join(chr(c) for c in dat),"99")
                b64 = base64.b64encode("".join(chr(c) for c in dat),"+/")
                l = str(len(b64))
                if first:
                    #First line contains direction
                    output += 'G7 @' +str(di)+ ' L' +l+ ' D' + b64 + '\n'
                    first = False
                else:
                    output += 'G7 L' +l+ ' D' + b64 + '\n'
            # output += G0(tox,y,"slow", "Raster data carrier") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
            return output
        
        def rastertoV(fromy, toy, x, data, di=3): # di = direction 3=positive (away from origin), 2=negative (towards origin)
            xscale = self.options.xscanline
            # inkex.errormsg("Rasterto: %s-%s, %s, %s, %s" % (fromx, tox, y, data, len(data)))
            # Break down data on multiple lines if necessary
            output=""
            # pixel = 0.09406 # Height of a pixel / row in raster image //TODO: Make option and calculate the correct export dpi automatically
            first=True
            d = get_chunks(data)

            # Work around: Correct the Half delay/pixel shift that occurs in the Marlin firmware.
            if di == 2:
                fromy += 1
            else:
                fromy += 0

            fromys = fromy / xscale

            #output += G0(x,fromys,"fast","Row n.%s blank space fast move" % y)

            output += G0(x,fromys,"slow",True,"Row n.%s" % y)
            #pixelsizel = float(pixelsize) / xscale
            #output += 'M649 S'+str(max_power)+' B2 D0 R'+str(pixelsizel)+'\n'
            for dat in d:
                #b64 = base64.b64encode("".join(chr(c) for c in dat),"99")
                b64 = base64.b64encode("".join(chr(c) for c in dat),"+/")
                l = str(len(b64))
                if first:
                    #First line contains direction
                    output += 'G7 @' +str(di)+ ' L' +l+ ' D' + b64 + '\n'
                    first = False
                else:
                    output += 'G7 L' +l+ ' D' + b64 + '\n'
            #output += G0(x,toy,"slow", "Raster data carrier") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
            ###output += G1(x,toy,"0") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
            return output

        def rasterto45(fromx, fromy, data, i, di=5): # di = direction 5= 45 deg -x +y, 4= +x -y
            # inkex.errormsg("Rasterto: %s-%s, %s, %s, %s" % (fromx, tox, y, data, len(data)))
            # Break down data on multiple lines if necessary
            output=""
            # pixel = 0.09406 # Height of a pixel / row in raster image //TODO: Make option and calculate the correct export dpi automatically
            first=True
            data.append(0)
            d = get_chunks(data)

            # Work around: Correct the Half delay/pixel shift that occurs in the Marlin firmware.
            fromx45 = fromx
            fromy45 = fromy
            if di == 4:
                fromx45 += 1
                fromy45 += 0
            else:
                fromx45 += 0
                fromy45 += 1

            pixelsize45 = float(pixelsize) * math.sqrt(2)
            if self.options.raster_45deg_adjust == True:
                pixelsize45 = pixelsize45 * math.sqrt(2)
                #fromx45 = fromx45 * math.sqrt(2)
                #fromy45 = fromy45 * math.sqrt(2)
            #output += G0(fromx45,fromy45,"slow","Row n.%s" % i)

            #output += 'M649 S'+str(max_power)+' B2 D0 R%.5f\n' % (pixelsize45)

            # new method. Fast move to the start or the data and use m649 to set the feedspeed of the raster
            
            output += G0(fromx45,fromy45,"fast",True,"Row n.%s fast move" % i)
            speed = F_G01
            speed = "{0:.1f}".format(self.feedratemod(x,y,x,y,speed)+ random.randint(-5,3))
            #output += 'M649 S'+str(max_power)+' B2 D0 R%.5f' % (pixelsize45)
            #output += ' F' + speed + '\n'
            for dat in d:
               
                #b64 = base64.b64encode("".join(chr(c) for c in dat),"99")
                b64 = base64.b64encode("".join(chr(c) for c in dat),"+/")
                l = str(len(b64))
                if first:
                    #First line contains direction
                    output += 'G7 @' +str(di)+ ' L' +l+ ' D' + b64 + '\n'
                    first = False
                else:
                    output += 'G7 L' +l+ ' D' + b64 + '\n'
            # output += G0(tox,y,"slow", "Raster data carrier") # G7 fills this move with raster data without this one the G0 that moves down a row gets the raster data.
            return output
        
        # Set the number of pixels/mm needed to accelerate up to rastering speed
        #accelspace = int(self.options.dummylength * scale) + 1
        # Add acceleration dummy moves:
        if (self.options.autodummy):
            feedmmpers = F_G01 / 60.0
            acceltime = feedmmpers / (2.0 * self.options.accel)
            acceldist = 2.0 * feedmmpers * acceltime * 1.1 # 1.1 is just to add a little buffer
            accel_length = round((acceldist*10.0)+0.5,0)/10.0
            raster_gcode += gccommentstart + 'Acceleration distance has been automatically calculated and set to %smm based on a feed of %smm/min and acceleration of %smm/s/s' % (accel_length, F_G01, self.options.accel) + gccommentend + '\n' 
        else:
            accel_length = self.options.dummylength
            raster_gcode += gccommentstart + 'Acceleration distance has been set to %smm' % (accel_length) + gccommentend + '\n' 
        
        accelspace = int(round(accel_length*scale*xscanline,0))

        if (self.options.burnwhite == True):
            maxwhite = 999999
        else:
            maxwhite = (accelspace * 2) + 2 # Maximum number of white pixels to allow in a raster command,
            if maxwhite < 20:
                maxwhite = 20

        y = 0
        x = 0 # Actual position in image, the farthest pixel that has been processed
        startx=0
        whitepixels = 10

        #pixelsizel = float(pixelsize) / xscanline
        #raster_gcode += 'M649 S'+str(max_power)+' B2 D0 R'+str(pixelsizel)+'\n'

        if (raster_dir == '45'):
            pixelsizel = float(pixelsize) * math.sqrt(2)
        else:
            pixelsizel = pixelsize

        raster_gcode += 'M649 S' + str(max_power) + ' B2 D0 R'+str(pixelsizel) + '\n'

        #if (self.options.raster_direction != '45'):
        inkex.errormsg("Acceleration Distance: %smm (%s pixels)" % (accel_length,accelspace))

        if (raster_dir != '45'):
            while y < len(grey_array):
                if y % 2 == 0:  # Back and forth motion, start by going right or Up
                    lastx = 0
                    startx = 0
                    while x < len(grey_array[y]):
                        # if pixel is not white: #(value less than white cut-off value)
                        if grey_array[y][x] > 0: #>= chr(255-self.options.white_cutoff):
                            # Look at the row of pixels to determine best approach; skip row if all white, break into smaller pieces if over maxwhite white pixels, else raster whole row
                            startx = x
                            whitepixels = 0
                            # loop until too many white pixels are found 
                            while x < len(grey_array[y]) and whitepixels <= maxwhite:
                                if grey_array[y][x] >0:#>= chr(255-self.options.white_cutoff):
                                    # Pixel is colourful, no problem, look for more pixels
                                    whitepixels = 0 # reset counter
                                    x += 1
                                    endx = x
                                else:
                                    # Pixel is boring white, count how many of those have been seen in a row
                                    whitepixels += 1
                                    x += 1
                            # x -= whitepixels # Only raster to the end of colourful pixels. x was advanced too much in the previous loop if it ended in too many white pixels.
                            # Finally, move to beginning of raster line and perform line.
                            startx1 = startx - accelspace
                            if (startx1 < 0):
                                startx1 = 0
                            if (startx1 < lastx):
                                startx1 = lastx+1
                            endx1 = endx + accelspace
                            lastx = endx1
                            data = grey_array[y][startx1:endx1]
                            data.append(0)
                            
                            data = rescale(data,min_power,max_power)

                            if raster_dir == "v":
                                direction = 3
                            else:
                                direction = 1
                            raster_gcode += rasterto(startx1,endx1,y,data,direction) # 1= direction: right
                                
                            
                        else: # Did not find a non-white pixel this time, continue right
                            x += 1
                else:  # Coming back left or Down
                    lastx = len(grey_array[y]) + accelspace
                    startx = -1
                    while x > 0:
                        # if pixel is not white: #(value less than white cut-off value)
                        if grey_array[y][x - 1] >0: #>= chr(255-self.options.white_cutoff):
                            # <= self.options.white_cutoff:
                            startx=x
                            whitepixels=0
                            while x > 0 and whitepixels <= maxwhite:
                                if grey_array[y][x-1] >0: #>= chr(255-self.options.white_cutoff):
                                    # Pixel is colourful, no problem, look for more pixels
                                    whitepixels = 0 # reset counter
                                    x -= 1
                                    endx = x
                                else:
                                    # Pixel is boring white, count how many of those have been seen in a row
                                    whitepixels += 1
                                    x -= 1
                                
                            # x += whitepixels # Only raster to the end of colourful pixels. x was advanced too much in the previous loop if it ended in too many white pixels.

                            # Finally, move to beginning of raster line and perform line.
                            startx1 = startx + accelspace
                            if (startx1 > lastx):
                                startx1 = lastx-1
                            endx1 = endx - accelspace
                            if (endx1 < 0):
                                endx1 = 0
                            lastx = endx1
                            data = grey_array[y][endx1:startx1]
                            data.insert(0,0)

                            data = rescale(data,min_power,max_power)

                            data.reverse()

                            if raster_dir == "v":
                                direction = 2
                            else:
                                direction = 0
                            raster_gcode += rasterto(startx1,endx1,y,data,direction) # 1= direction: left
    
                        else:
                            x -= 1
              
                # At the end of the row, move down by one pixel
                y += 1
        else: #45 degree stuff
            # Using a grid with index points around he edge. eg in a 5x4 grid
            #  0   1   2   3   4    5
            # 0+---+---+---+---+---+
            #  !   !   !   !   !   !
            # 1+---+---+---+---+---+6
            #  !   !   !   !   !   !
            # 2+---+---+---+---+---+7
            #  !   !   !   !   !   !
            # 3+---+---+---+---+---+8
            #  !   !   !   !   !   !
            # 4+---+---+---+---+---+
            #      5   6   7   8    9

            #  0     1     2 
            # 0+-----+-----+---~
            #  !    /!    /! 
            #  !  /  !  /  ! 
            #  !/    !/    ! 
            # 1+-----+-----+---~
            #  !    /!     ! 
            #  !  /  !     ! 
            #  !/    !     ! 
            # 2+-----+-----+---~
            #  !     !     ! 
            # 3+-----+-----+---~
            #  !     !     ! 
            # 4+-----+-----+---~
            #        5     6 
            # Diagonal lines will always run from the same index number on group

            # the modified marlin firmware, Direction 4 goes -x +y direction and direction 5 foes +x -y
            
            i = 1 # i is index
            # len(grey_array) is length of Y or Height, len(grey_array[0]) is length of X or width
            w = len(grey_array[0])
            h = len(grey_array)
            #inkex.errormsg("w:%s" % (w))
            #inkex.errormsg("h:%s" % (h))
            #inkex.errormsg("w+h:%s" % (w+h))

            #whitespace = int(5.0 * scale)
            #whitespace = round(accel_length * scale,0) + 1
            whitespace = int(accelspace + 1)

            while i < (w+h) : 
                
                # Get X and Y cordinated for X along the Origin line and Y furthest from the origin
                if i < w:
                    x1 = i
                    y1 = 0
                    px1 = x1-1
                    py1 = 0
                else:
                    x1 = w
                    y1 = i-w
                    px1 = x1-1
                    py1 = y1-1
                # Get X and Y cordinated for X furthest from the Origin and Y along the origin line
                if i < h:
                    x2 = 0
                    y2 = i
                    px2 = 0
                    py2 = y2-1
                else:
                    x2 = i-h
                    y2 = h
                    px2 = x2
                    py2 = y2-1
                # Calculate the length on the line in pixels at the given index
                diag_length = min(w,h,(w+h)-i,i)

                # Define and fill the array containing the diagonal data.
                diag_array = []
                for k in range(diag_length):
                    diag_array.append(0)                
                
                # Set Start and End coorinates
                if ((i-1) % 2) == 0: # start with Direction 4 -X +Y
                    first_pix_x = px1 # first_pix is the starting pixel in the grey_array
                    first_pix_y = py1
                    start_cord_x = x1 # start_cord is the starting coordinate on the engraver 
                    start_cord_y = y1 #
                else: # start location for direction +X -Y
                    first_pix_x = px2
                    first_pix_y = py2
                    start_cord_x = x2
                    start_cord_y = y2

                #START OF NEW CODE
                j = 0
                whitecount = 0
                while j < diag_length:
                    if ((i-1) % 2) == 0: # start with Direction 4 -X +Y
                        diag_array[j] = grey_array[first_pix_y+j][first_pix_x-j]
                    else:
                        diag_array[j] = grey_array[first_pix_y-j][first_pix_x+j]
                    j += 1

                # At this point we now have the starting point X, Y and the data for the line

                # Now lets start scanning the data and flag the first none blank pixel we find 
                first_burn = 0 #diag_length-1
                #inkex.errormsg("diag_length:%s" % diag_length)
                j = 0
                indata = False # this is set to true once a none blank is found and is set back to false one a section has been coded
                while j < diag_length:
                    #if diag_length == 100:
                        #inkex.errormsg("[%s] = %s" % (j,diag_array[j]))
                    if diag_array[j] > (255-self.options.white_cutoff):
                        if indata == False:
                            first_burn = j
                            indata = True
                        whitecount = 0
                            #j = diag_length
                    else:
                        whitecount += 1
                        
                    if indata == True:
                        if (whitecount > (whitespace*2)) or (j >= (diag_length-1)):
                            # we have a valid chunk to write out, now pad and start and end for accelleration and encode items
                            first_burn = max(0, first_burn-whitespace) # take first pixel 2 pixels back from the start, but not less than 0
                            if j >= (diag_length-1):
                                #last_burn = diag_length-1
                                last_burn = (diag_length-1) - (whitecount-whitespace) + 1
                                last_burn = min(last_burn,diag_length-1)
                            else:
                                last_burn = min(diag_length-1, j-whitespace) # take last pixel 2 pixels back from the end, but not greater than the max

                            if ((i-1)%2) == 0:
                                new_start_cord_x = start_cord_x - first_burn
                                new_start_cord_y = start_cord_y + first_burn
                                new_end_cord_x = start_cord_x - last_burn #- 1
                                new_end_cord_y = start_cord_y + last_burn #+ 1
                            else:
                                new_start_cord_x = start_cord_x + first_burn
                                new_start_cord_y = start_cord_y - first_burn
                                new_end_cord_x = start_cord_x + last_burn #+ 1
                                new_end_cord_y = start_cord_y - last_burn #- 1

                            new_diag_length = (last_burn+1)-first_burn

                            data = []
                            for k in range(new_diag_length):
                                data.append(0)                

                            #new_diag_array = diag_array[first_burn:last_burn]
                            k = 0
                            while k < new_diag_length:
                                data[k] = diag_array[k+first_burn]
                                k += 1

                            data = rescale(data,min_power,max_power)

                            raster_gcode += rasterto45(new_start_cord_x, new_start_cord_y, data, i, 5-(i%2)) # 1= direction: right
                            #raster_gcode += G0(new_end_cord_x,new_end_cord_y,"slow","Row end")
                            #raster_gcode += G1(new_end_cord_x,new_end_cord_y,"0")
                            
                            indata = False
                            whitecount = 0
                            
                            
                    j += 1
                i += 1
                #END OF NEW CODE
        raster_gcode += 'M649 S0 B0 D0 P0\n' # Trying to turn off the laser
        raster_gcode += 'M5\n'
        return raster_gcode

    
    
    # Generate gcode that follows svg paths
    def generate_gcode(self, curve, laserPower, altfeed, altppm, repeat, line_type, lpwmm, lplmm):
        gcode = ''

        # Setup feed rate
        cutFeed = "F%s" % altfeed

        # Setup our pulse per millimetre option, if applicable
        # B: laser firing mode (0 = continuous, 1 = pulsed, 2 = raster)

        if self.options.mainboard == "marlin":
            if (line_type == "s"):
            # Use the "alternative" ppm - L60000 is 60us
                ppmValue = "B0 D0"
            else:
                inkex.errormsg("line_type:%s, lpwmm:%s, lplmm:%s, altfeed:%s" % (line_type, lpwmm, lplmm, altfeed))
                ppm = 1.0/lpwmm
                pdur = ((1.0/(altfeed/60))*lplmm) * 1000000
                #ppmValue = "L60000 P%.2f B1 D0" % altppm
                ppmValue = "L%.0f P%.4f B1 D0" % (pdur, ppm)
        else:
            ppmValue = ""

        #if (altppm):
            ## Use the "alternative" ppm - L60000 is 60us
            #if self.options.mainboard == "grbl":
                #ppmValue = ""
            #else:
                #ppmValue = "L60000 P%.2f B1 D0" % altppm
        #else:
            ## Set the laser firing mode to continuous.
            #if self.options.mainboard == "grbl":
                #ppmValue = ""
            #else:
                #ppmValue = "B0 D0"

        cwArc = "G2"
        ccwArc = "G3"

        # The geometry is reflected, so invert the orientation of the arcs to match
        if (self.flipArcs):
            (cwArc, ccwArc) = (ccwArc, cwArc)

        # The 'laser on' and 'laser off' m-codes get appended to the GCODE generation
        lg = 'G0'
        firstGCode = False

        newstuff = ''  # For repeating paths for efficient cutting, this stores the current shape to repeat after for loop completes

        for i in range(1, len(curve['data'])):
            s, si = curve['data'][i - 1], curve['data'][i]
            #inkex.errormsg(str(s))
            #inkex.errormsg(str(i) +': ' +str(si))
            # x1: si[0][0]
            # intens(si[0][0],si[0][1],si[4][0],si[4][1],laserPower)
            # G0 : Move with the laser off to a new point
            if s[1] == 'move':
                # Turn off the laser if it was on previously.
                # if lg != "G0":
                #    gcode += LASER_OFF + "\n"

                # Disarm laser before a G0 Move (Not needed for GRBL in laser mode or MK4Duo in Laser mode
                newstuff += laserdisablecmd + "\n"
                newstuff += "G0 " + self.make_args(si[0]) + " F%i " % self.options.Mfeed + "\n"
                newstuff += laserenablecmd + "\n"
                lg = 'G0'

            elif s[1] == 'end':
                lg = 'G0'

            # G1 : Move with the laser turned on to a new point
            elif s[1] == 'line':
                # Correct laser intensity:
                # laserPower = self.intens(si[0][0],si[0][1],0,0,laserPower)
                cutFeed = 'F'+str(int(self.feedratemod(si[0][0],si[0][1],0,0,int(altfeed))))
                if not firstGCode:  # Include the ppm values for the first G1 command in the set.
                    newstuff += "G1 " + self.make_args(
                        si[0]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"
                    firstGCode = True
                else:
                    newstuff += "G1 " + self.make_args(
                        si[0]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"
                    # newstuff += "G1 " + self.make_args(si[0]) + "\n"
                lg = 'G1'

            # G2 and G3 : Move in an arc with the laser turned on.
            elif s[1] == 'arc':
                #inkex.errormsg(str(i) +': ' +str(si))
                cutFeed = 'F'+str(int(self.feedratemod(si[0][0],si[0][1],0,0,int(altfeed))))
                #laserPower = self.intens(si[0][0],si[0][1],0,0,laserPower)
                dx = s[2][0] - s[0][0]
                dy = s[2][1] - s[0][1]
                if abs((dx ** 2 + dy ** 2) * self.options.Xscale) > self.options.min_arc_radius:
                    r1 = P(s[0]) - P(s[2])
                    r2 = P(si[0]) - P(s[2])
                    if abs(r1.mag() - r2.mag()) < 0.001:
                        if (s[3] > 0):
                            newstuff += cwArc
                        else:
                            newstuff += ccwArc

                        if not firstGCode:  # Include the ppm values for the first G1 command in the set.
                            newstuff += " " + self.make_args(si[0] + [None, dx, dy,
                                None]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"
                            firstGCode = True
                        else:
                            # newstuff += " " + self.make_args(si[0] + [None, dx, dy, None]) + "\n"
                            newstuff += " " + self.make_args(si[0] + [None, dx, dy,
                                None]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"

                    else:
                        r = (r1.mag() + r2.mag()) / 2
                        if (s[3] > 0):
                            newstuff += cwArc
                        else:
                            newstuff += ccwArc

                        if not firstGCode:  # Include the ppm values for the first G1 command in the set.
                            newstuff += " " + self.make_args(si[0]) + " R%f" % (
                                r * self.options.Xscale) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"
                            firstGCode = True
                        else:
                            # newstuff += " " + self.make_args(si[0]) + " R%f" % (r*self.options.Xscale) + "\n"
                            newstuff += " " + self.make_args(si[0]) + " R%f" % (
                                r * self.options.Xscale) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"

                    lg = cwArc
                # The arc is less than the minimum arc radius, draw it as a straight line.
                else:
                    if not firstGCode:  # Include the ppm values for the first G1 command in the set.
                        newstuff += "G1 " + self.make_args(
                            si[0]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"
                        firstGCode = True
                    else:
                        # newstuff += "G1 " + self.make_args(si[0]) + "\n"
                        newstuff += "G1 " + self.make_args(
                            si[0]) + " S%.1f " % laserPower + "%s " % cutFeed + "%s" % ppmValue + "\n"

                    lg = 'G1'
        for rep in range(int(repeat)):
            gcode += "\n; Repeat: %s" % int(rep + 1) + " of %s\n" % int(repeat)
            if self.options.mainboard == "grbl":
                gcode += self.options.m3commands + " S0" + "\n" + newstuff
            else:
                gcode += "M3 S0" + "\n" + newstuff


        # The end of the layer.
        if si[1] == 'end':
            gcode += LASER_OFF
            # gcode += "G28 \n"

        return gcode

    def tool_change(self):
        # Include a tool change operation, for allowing the user to turn the workpiece around
        gcode = TOOL_CHANGE % (self.currentTool + 1)
        # Select the next available tool
        self.currentTool = (self.currentTool + 1) % 32
        return gcode

    # Determine the tmp directory for the user's operating system.
    def getTmpPath(self):
        """Define the temporary folder path depending on the operating system"""

        if os.name == 'nt':
            return 'C:\\\\Temp\\'
            #return 'C:\\WINDOWS\\Temp\\'
        else:
            return '/tmp/'

    ################################################################################
    ###
    ###        Curve to Gcode
    ###
    ################################################################################


    def effect_curve(self):

        def gccomment(gc):
            if (self.options.gcodecomments): # remove gcode comments
                enablecomments = False
            else:
                enablecomments = True

            sgc = '\n'
            if (gc == ''):
                sgc = sgc 
            else:
                if (enablecomments):
                    sgc = ' ' + gccommentstart + gc + gccommentend + sgc
            return sgc
            
        self.skipped = 0
        current_file = self.args[-1]
        # Recursively compiles a list of paths that are descendant from the given node
        def compile_paths(parent, node, trans):
            # Apply the object transform, along with the parent transformation
            mat = node.get('transform', None)
            path = {}

            if mat:
                mat = simpletransform.parseTransform(mat)
                trans = simpletransform.composeTransform(trans, mat)

            if node.tag == SVG_PATH_TAG:
                # This is a path object
                if (not node.get("d")): return []
                csp = cubicsuperpath.parsePath(node.get("d"))

                path['type'] = "vector"
                path['id'] = node.get("id")
                path['data'] = []

                if (trans):
                    simpletransform.applyTransformToPath(trans, csp)
                    path['data'] = csp

                # Apply a transform in the Y plan to flip the path vertically
                # If we want our origin to the the top left.
                if (self.options.origin == 'topleft'):
                    csp = path['data']
                    simpletransform.applyTransformToPath(([1.0, 0.0, 0], [0.0, -1.0, 0]), csp)
                    path['data'] = csp

                return path

            elif node.tag == SVG_GROUP_TAG:
                # This node is a group of other nodes
                pathsGroup = []
                for child in node.iterchildren():
                    data = compile_paths(parent, child, trans)
                    # inkex.errormsg(str(data))
                    if type(data) is not list:
                        pathsGroup.append(data.copy())
                    else:
                        pathsGroup += data
                return pathsGroup
            
            #TODO: Raster objects that are not paths even if raster was not specified in the layer params? Or convert to paths? (Temporarily, don't want t o affect the svg file)

        # first find only visible layers
        visible_layers = []
        #inkex.errormsg(str(self.document))
        for layer in get_layers(self.document):
            #inkex.errormsg(layer.get('id'))
            #inkex.errormsg(str(layer))
            visible = True
            try: 
                # The style category is not necessarily set for a new layer, so we need to be careful with checking for display:none
                if 'display:none' in layer.get('style'): 
                    visible = False
            except:
                pass
            if visible:
                visible_layers.append(layer)    
                

        # Decide what to cut and how, and write gcode
        gcode = ""
        gcode_raster = ""
        for layer in visible_layers:
            label = layer.get(SVG_LABEL_TAG).strip()
            pathList = []

            # First get layerparams
            try:
                layerParams = parse_layer_name(label)
                
            except ValueError, e:
                inkex.errormsg("Your inkscape layer \"%s\" is named incorrectly, can't figure out what to do with it!" % label)
                return
                
            # Check that laser intensity correction has small enough power setting to operate sensibly:
            #if layerParams.get('power',self.options.laser_max_value) > (100/(1.0+INTENSITY_CORRECTION)):
            #    inkex.errormsg("Laser intensity for layer \"%s\" is too high for intensity correction to operate sensibly. With current settings, the maximum value is %s." % (label,"{0:.1f}".format(100/(1.0+INTENSITY_CORRECTION))))
            # inkex.errormsg("rasterfodrhkop" + layerParams['raster'])
            # Apply the layer transform to all objects within the layer
            trans = layer.get('transform', None)
            trans = simpletransform.parseTransform(trans)
            
            # Decision time
            if layerParams.get('raster', False):
                # Raster this layer
                raster_max_power = layerParams.get('power',self.options.laser_max_value)
                raster_max_power = layerParams.get('maxpower',raster_max_power)
                raster_min_power = layerParams.get('minpower',self.options.laser_min_value)
                raster_dir = layerParams.get('dir',self.options.raster_direction)
                raster_greyscale = layerParams.get('grey',self.options.raster_greyscale)
                if (self.options.raster_method == "base64") and (self.options.mainboard == "marlin"):
                    inkex.errormsg("Will raster layer " + layer.get('id') + " using base64 encoding.")
                    gcode_raster += gccommentstart + "Rastering layer " + layer.get('id') + ': ' + label + gccommentend + ' using base64 encoding\n'

                    gcode_raster += self.Rasterbase64(layer.get("id"), layerParams.get('feed',self.options.speed_ON), layerParams.get('resolution',self.options.resolution),
                    raster_max_power,raster_min_power,raster_dir,raster_greyscale)
                else:
                    inkex.errormsg("Will raster layer " + layer.get('id') + " using indivdual gcodes.")
                    gcode_raster += gccommentstart + "Rastering layer " + layer.get('id') + ': ' + label + gccommentend + ' using individual gcodes\n'

                    gcode_raster += self.Raster(layer.get("id"), layerParams.get('feed',self.options.speed_ON), layerParams.get('resolution',self.options.resolution),
                    raster_max_power,raster_min_power,raster_dir,raster_greyscale)
                
            elif layerParams.get('crosshatch', False):
                # Raster by cross-hatching diagonally
                inkex.errormsg("Will diagonally cross-hatch raster layer " + layer.get('id'))
                gcode_raster += gccommentstart + "Diagonally rastering layer " + layer.get('id') + ': ' + label + gccommentend + '\n'

                gcode_raster += self.Crosshatch(layer.get("id"), layerParams.get('feed',self.options.speed_ON), layerParams.get('resolution',self.options.resolution),
                layerParams.get('power',self.options.laser_max_value))
            else:
                # cut as vector, recursively if needed
                inkex.errormsg("Will cut layer " + layer.get('id'))
                for node in layer.iterchildren():
                    try:
                        pathList.append(compile_paths(self, node, trans).copy())
                    except:
                        try:
                            for objectData in compile_paths(self, node, trans):
                                pathList.append(objectData)
                        except:
                            inkex.errormsg("Layer %s seems to be empty." % layer.get('id'))

                # paths to be cut are now in pathlist
                for objectData in pathList:
                    curve = self.parse_curve(objectData)
                    gcode += gccommentstart + "Cutting layer " + layer.get('id') + ': ' + label + gccommentend + '\n'
                    gcode += self.generate_gcode(curve, 
                        layerParams.get('power',self.options.laser), 
                        layerParams.get('feed',self.options.feed), 
                        layerParams.get('ppm',None), 
                        layerParams.get('repeat',1),
                        layerParams.get('line_type',self.options.line_type),
                        layerParams.get('lpwmm',self.options.lpwmm),
                        layerParams.get('lplmm',self.options.lplmm))
        
        # HOMING after everything has been cut
        if self.options.homeafter:
            # gcode += "\n\nG0 X0 Y0 F4000 ; home\n"
            if self.options.homing == 1:
                gcode += 'G28 ' + gccomment('home all axes')
            elif self.options.homing == 2:
                gcode += '$H ' + gccomment('home all axes')
            elif self.options.homing == 3:
                gcode += ('G00 X0 Y0 F%s ' % self.options.Mfeed) + gccomment('Returning to origin')
            else:
                pass

        # Now write gcode, raster data first
        return gcode_raster + "\n\n" + gcode

    def effect(self):
        global options
        options = self.options
        #selected = self.selected.values()

        global laserenablecmd
        global laseroffcmd
        global laserpwrcmd
        global laserdisablecmd
        global laser_combine_move_and_power
        global laser_G0_Burns
        global gccommentstart
        global gccommentend
        global lasermaxpower
		
        root = self.document.getroot()
        # TODO: This does not seem to affect anything? Maybe used to be useful for the old rastering method?
        # See if the user has the document setup in mm or pixels.
        try:
            self.pageHeight = float(root.get("height", None))
        except:
            inkex.errormsg((
                "Please change your inkscape project units to be in pixels, not inches or mm. In Inkscape press ctrl+shift+d and change 'units' on the 'Custom size' tab to px. The option 'default units' can be set to mm or inch, these are the units displayed on your rulers."))
            return

        self.flipArcs = (self.options.Xscale * self.options.Yscale < 0)
        self.currentTool = 0

        self.filename = options.file.strip()
        if (self.filename == "-1.0" or self.filename == ""):
            inkex.errormsg(("Please select an output file name."))
            return
        
        if (not self.filename.lower().find(".")): # endswith(GCODE_EXTENSION)):
            # Automatically append the correct extension, if no extension is specified
            self.filename += GCODE_EXTENSION

        logger.enabled = self.options.logging
        logger.write("Laser script started")
        logger.write("output file == %s" % self.options.file)

        # if len(selected) <= 0:
            # inkex.errormsg(("This extension requires at least one selected path."))
            # return

        dirExists = self.check_dir()
        if (not dirExists):
            return

        # TODO: Working on a number of different controller types. 
        #<item value="marlin">Marlin + Ramps 1.4</item>
        #<item value="grbl">GRBL 1.1</item>
        #<item value="grbllpc">GRBL 1.1 on Smoothie</item>
        #<item value="uccnc">UCCNC</item>
        #<item value="smoothie">Smoothie</item>

        if (self.options.mainboard == 'grbl'): #gcode for GRBL
            laserenablecmd = self.options.m3commands
            laseroffcmd = "S0"
            laserpwrcmd = "S"
            laserdisablecmd = "M5"
            laser_combine_move_and_power = True
            laser_G0_Burns = False
            gccommentstart = ";"
            gccommentend = ""
            lasermaxpower = 1000
        else:
            if (self.options.mainboard == 'grbllpc'): # same as GRBL but with smoothie power range
                laserenablecmd = self.options.m3commands
                laseroffcmd = "S0"
                laserpwrcmd = "S"
                laserdisablecmd = "M5"
                laser_combine_move_and_power = True
                laser_G0_Burns = False
                gccommentstart = ";"
                gccommentend = ""
                lasermaxpower = 1
            else:
                if (self.options.mainboard == 'smoothie'): # gcode for Smoothie and Marlin. Same as GRBL but no support for M4 Laser mode.
                    laserenablecmd = "M3"
                    laseroffcmd = "S0"
                    laserpwrcmd = "S"
                    laserdisablecmd = "M5"
                    laser_combine_move_and_power = True
                    laser_G0_Burns = False
                    gccommentstart = ";"
                    gccommentend = ""
                    lasermaxpower = 1
                else:
                    if (self.options.mainboard == 'marlin'): # gcode for Smoothie and Marlin. Same as GRBL but no support for M4 Laser mode.
                        laserenablecmd = "M3"
                        laseroffcmd = "S0"
                        laserpwrcmd = "S"
                        laserdisablecmd = "M5"
                        laser_combine_move_and_power = True
                        laser_G0_Burns = False
                        gccommentstart = ";"
                        gccommentend = ""
                        lasermaxpower = 100
                    else:
                        if (self.options.mainboard == 'uccnc'): # gcode for UCCNC
                            laserenablecmd = "M10"
                            laseroffcmd = "Q0"
                            laserpwrcmd = "Q"
                            laserdisablecmd = "M11"
                            laser_combine_move_and_power = False
                            laser_G0_Burns = True
                            gccommentstart = "( "
                            gccommentend = " )"
                            lasermaxpower = 255


        LASER_ON = laserenablecmd + " ;turn the laser on"  # LASER ON MCODE
        LASER_OFF = laserdisablecmd + " ;turn the laser off\n"  # LASER OFF MCODE
			
        def gccomment(gc, enablecommentsoverride = False):
            if (self.options.gcodecomments): # remove gcode comments
                enablecomments = enablecommentsoverride
            else:
                enablecomments = True

            sgc = '\n'
            if (gc == ''):
                sgc = sgc 
            else:
                if (enablecomments):
                    sgc = ' ' + gccommentstart + gc + gccommentend + sgc
            return sgc

        gcode = self.header
        gcode += gccommentstart + "This gcode was generated with Metchit Laser Engraver Exporter:" + gccommentend + "\n"
        gcode += gccommentstart + "A powerful Inkscape extension that can make gcode for both cutting paths" + gccommentend + "\n"
        gcode += gccommentstart + "and rastering/engraving images. Brought to you by John Revill at Metchit." + gccommentend + "\n"
        # Save svg document name into gcode file so you'll know what the file contains
        try:
            SODIPODI_NAMESPACE = "http://sodipodi.sourceforge.net/DTD/sodipodi-0.dtd"
            SODIPODI = "{%s}" % SODIPODI_NAMESPACE
            NSMAP = {'sodipodi' : SODIPODI_NAMESPACE}
            
            #inkex.errormsg(str(root.get(SODIPODI + "docname")))
            gcode += gccommentstart + "This gcode is based on the file: " + str(root.get(SODIPODI + "docname")) + gccommentend + "\n"  
        except:
            pass
        # TODO: if statement that tells you what firmware this file is meant for.
        if (self.options.mainboard == 'grbl'):
            gcode += gccommentstart + "This gcode is optimized for GRBL 1.1f in Laser mode only and may fail on other firmwares." + gccommentend + "\n"
        elif (self.options.mainboard == 'uccnc'):
            gcode += gccommentstart + "This gcode is optimized for UCCNC laser cutters and may fail on other firmwares." + gccommentend + "\n"
        elif (self.options.mainboard == 'smoothie'):
            gcode += gccommentstart + "This gcode is optimized for smoothieware CNC boards for Lasers and may fail on other firmwares." + gccommentend + "\n"
        elif (self.options.mainboard == 'marlin'):
            gcode += gccommentstart + "This gcode is optimized for Marlin with modifications for laser cutters and will fail on other firmwares." + gccommentend + "\n"
        #gcode += gccommentstart + "If your G-code sender application does not support GRBL special commands '$H for example' features like homing will fail." + gccommentend + "\n"
        if (self.options.unit == "mm"):
            self.unitScale = 0.282222222222
            if (self.options.mainboard <> 'uccnc'):
                gcode += "G21" + gccomment('All units in mm', True) 
        elif (self.options.unit == "in"):
            self.unitScale = 0.011111
            if (self.options.mainboard <> 'uccnc'):
                gcode += "G20" + gccomment('All units in inches', True)
        else:
            inkex.errormsg(("You must choose mm or in"))
            return

        gcode += 'G90' + gccomment('Use absolute coordinates', True)

        if (self.options.mainboard == "marlin"):
            gcode += 'M80' + gccomment('Turn on Optional Peripherals Board at LMN', True)
            #gcode += 'M106 S255' + gccomment('Turn on blower fan at 100%', True)


        # Put the header data in the gcode file
        gcode2 = """
; Raster data will always precede vector data
; Default Cut Feedrate %i mm per minute
; Default Move Feedrate %i mm per minute
; Default Laser Intensity %i percent
; Default Laser Raster Minimum Intensity %i percent
; Default Laser Raster Maximum Intensity %i percent\n""" % (self.options.feed, self.options.Mfeed, self.options.laser,self.options.laser_min_value,self.options.laser_max_value)

        gcode += gccommentstart + " Raster data will always precede vector data" + gccommentend + "\n"
        gcode += gccommentstart + " Default Cut Feedrate %i mm per minute" % (self.options.feed) + gccommentend + "\n"
        gcode += gccommentstart + " Default Move Feedrate %i mm per minute" % (self.options.Mfeed) + gccommentend + "\n"
        gcode += gccommentstart + " Default Laser Intensity %i percent" % (self.options.laser) + gccommentend + "\n"
        gcode += gccommentstart + " Default Laser Raster Minimum Intensity %i percent" % (self.options.laser_min_value) + gccommentend + "\n"
        gcode += gccommentstart + " Default Laser Raster Maximum Intensity %i percent" % (self.options.laser_max_value) + gccommentend + "\n"

        if self.options.homebefore:
            #gcode += "G28 ; home all\n\n"
            if self.options.homing == 1:
                gcode += 'G28 ' + gccommentstart + 'home all axes' + gccommentend + '\n'
            elif self.options.homing == 2:
                gcode += '$H ' + gccommentstart + 'home all axes' + gccommentend + '\n'
            elif self.options.homing == 3:
                gcode += ('G00 X0 Y0 F%s ' % self.options.Mfeed) + gccommentstart + 'Returning to origin' + gccommentend + '\n'
            else:
                pass

        gcode += gccommentstart + "========== END OF HEADER ========" + gccommentend + "\n"


        # if self.options.function == 'Curve':
        data = self.effect_curve() #selected)
        if data:
            gcode += data
            gcode += "M5\n"
            if (self.options.mainboard == "uccnc"):
                gcode += "M11\n"
            if (self.options.mainboard == "marlin"):
                gcode += "M107 " + gccommentstart + "Turn off blower fan" + gccommentend + '\n'
                gcode += "M03 S0.1 " + gccommentstart + "Set laser to dim alignment beam" + gccommentend + '\n'

        # if (self.options.double_sided_cutting):
            # gcode += "\n\n;(MSG,Please flip over material)\n\n"
            # # Include a tool change operation
            # gcode += self.tool_change()

            # logger.write("*** processing mirror image")

            # self.options.Yscale *= -1
            # self.flipArcs = not (self.flipArcs)
            # # self.options.generate_not_parametric_code = True
            # self.pageHeight = 0
            # gcode += self.effect_curve()#selected)

        try:
            f = open(self.options.directory + '/' + self.options.file, "w")
            f.write(gcode + self.footer)
            f.close()
        except:
            inkex.errormsg(("Can not write to specified file!"))
            return

        if (self.skipped > 0):
            inkex.errormsg((
                "Warning: skipped %d object(s) because they were not paths (Vectors) or images (Raster). Please convert them to paths using the menu 'Path->Object To Path'" % self.skipped))


e = Gcode_tools()
e.affect()
inkex.errormsg("Finished processing.")
