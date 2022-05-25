# Copyright (c) 2021, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
#
# All rights reserved.
#
# The "ISAAC - Integrated System for Autonomous and Adaptive Caretaking
# platform" software is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

# Create camera files based on undistorted images that ASP will understand.

import argparse
import glob
import os
import re
import sys

import numpy as np

parser = argparse.ArgumentParser(
    description="Convert cameras to the format of texrecon."
)
parser.add_argument(
    "--camera_dir",
    default="",
    help="The directory containing the camera information (the output of geometry_mapper).",
)
parser.add_argument(
    "--undistorted_image_dir",
    default="",
    help="The directory containing the undistorted images.",
)
parser.add_argument(
    "--camera_type",
    default="",
    help="The camera type (nav_cam, haz_cam, or sci_cam, etc.).",
)

args = parser.parse_args()

if args.camera_dir == "" or args.undistorted_image_dir == "" or args.camera_type == "":
    print(
        "Must specify the camera directory, directory of undistorted images, and camera type."
    )
    sys.exit(1)

# Read the intrinsics
intr_file = args.undistorted_image_dir + "/undistorted_intrinsics.txt"
if not os.path.exists(intr_file):
    print("Missing file: " + intr_file)
    sys.exit(1)
with open(intr_file, "r") as f:
    for line in f:
        if re.match("^\s*\#", line):
            continue  # ignore the comments
        vals = line.split()
        if len(vals) < 5:
            print("Expecting 5 parameters in " + intr_file)
            sys.exit(1)
        widx = float(vals[0])
        widy = float(vals[1])
        f = float(vals[2])
        cx = float(vals[3])
        cy = float(vals[4])

        break  # finished reading the line we care for

# Read the undist images
undist_images = glob.glob(args.undistorted_image_dir + "/*.jpg")
for image in undist_images:
    base = os.path.basename(image)
    base = os.path.splitext(base)[0]
    cam2world_file = args.camera_dir + "/" + base + "_cam2world.txt"

    if not os.path.exists(cam2world_file):
        # Not all images are in the map and hence have a cam2world file
        continue
    
    # Load the cam2world transform
    M = np.loadtxt(cam2world_file)

    # Write the TSAI file
    tsai_file = args.undistorted_image_dir + "/" + base + ".tsai"
    print("Writing: " + tsai_file)
    with open(tsai_file, "w") as g:
        g.write("VERSION_3\n")
        g.write("fu = %0.17g\n" % f)
        g.write("fv = %0.17g\n" % f)
        g.write("cu = %0.17g\n" % cx)
        g.write("cv = %0.17g\n" % cy)
        g.write("u_direction = 1 0 0\n")
        g.write("v_direction = 0 1 0\n")
        g.write("w_direction = 0 0 1\n")
        g.write("C = %0.17g %0.17g %0.17g\n" % (M[0][3], M[1][3], M[2][3]))
        g.write("R = %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g %0.17g\n" %
                (M[0][0], M[0][1], M[0][2],
                 M[1][0], M[1][1], M[1][2],
                 M[2][0], M[2][1], M[2][2]))
        g.write("pitch = 1\n")
        g.write("NULL\n")
