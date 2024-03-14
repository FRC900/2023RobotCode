#!/bin/bash


current_dir=$(dirname "$0")
#gets current dir ^


cd "$current_dir/../zebROS_ws/src" || exit


git submodule init
git submodule update
git lfs install
git lfs pull

echo "Inside of docker, navigate to 2023Robotcode/zebROS_ws/src, and run: "wstool update""
