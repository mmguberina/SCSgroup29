#!/bin/bash

ffmpeg -f x11grab -s 1920x1080 -i :0 -f alsa -i default testing_explanation.mkv
