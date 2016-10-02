#!/bin/sh

ffmpeg -r 0.7 -f image2 -i snap%d.png -s 1000x1000 -y simulation.avi
