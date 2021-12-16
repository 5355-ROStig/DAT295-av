#!/usr/bin/env python3
import numpy as np
import array
import math
from datetime import datetime, timedelta
import collections
from kalman import Kalman
from median import Median
from split_median import SplitMedian
import argparse
import csv

import robot


if __name__ == '__main__':
    parser = argparse.ArgumentParser("gv_ttc")

    parser.add_argument("input", help="File to read from")
    parser.add_argument("output", help="File to save to")
    parser.add_argument("-f", "--filter", help="Filter to use (kalman, median, split_median)", default="median")
    parser.add_argument("-s", "--median-samples", help="Number of samples to use for means/medians", type=int, default=4)
    args = parser.parse_args()

    # Choose filtering method
    if args.filter == 'kalman':
        robot.filter_creator = lambda initPos : Kalman(initPos)
    elif args.filter == 'median':
        robot.filter_creator = lambda initPos : Median(initPos, args.median_samples)
    elif args.filter == 'split_median':
        robot.filter_creator = lambda initPos : SplitMedian(initPos, args.median_samples)
    else:
        sys.exit("Invalid filter type")
    print("Using " + args.filter + " filter ")
    
    with open(args.input, newline='') as input:
        with open(args.output, "w", newline='') as output:
            writer = csv.writer(output, dialect='excel')
            writer.writerow(["time", "tag", "x", "y", "vx", "vy", "ttc"]) # Write header
            reader = csv.DictReader(input)
            
            starttime = -1
            
            for row in reader:
                time = float(row['time'])
                x = float(row['x'])
                y = float(row['y'])
                tag = int(row['tag'])
                
                r = robot.receive_pos(int(tag), float(x), float(y), float(time))
                ttc = r.collisionCheck()
                
                if starttime == -1:
                    starttime = time
                
                writer.writerow([time-starttime, tag, r.p[0], r.p[1], r.v[0], r.v[1], ttc])

    print("Bye!")
