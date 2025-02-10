#!/bin/bash

cd /home/chenyang/repos/tmp/CGGS-CW1-Geometry

git pull --recurse-submodules

cp /home/chenyang/repos/CGGS-CW1-Geometry/code/include/compute* /home/chenyang/repos/tmp/CGGS-CW1-Geometry/code/include/

cp -r /home/chenyang/repos/tmp/CGGS-CW1-Geometry/code/ /home/chenyang/repos/CGGS-CW1-Geometry/
cp -r /home/chenyang/repos/tmp/CGGS-CW1-Geometry/data/ /home/chenyang/repos/CGGS-CW1-Geometry/
cp /home/chenyang/repos/tmp/CGGS-CW1-Geometry/*.pdf /home/chenyang/repos/CGGS-CW1-Geometry/
cp /home/chenyang/repos/tmp/CGGS-CW1-Geometry/*.md /home/chenyang/repos/CGGS-CW1-Geometry/

git stash