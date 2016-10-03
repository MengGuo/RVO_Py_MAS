#!/bin/sh

ffmpeg -r 3 -f image2 -i snap%d.png -s 1000x1000 -y simulation.avi
