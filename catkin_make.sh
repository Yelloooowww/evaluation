#! /bin/bash

if [ "$1" ]; then
    echo "limiting compiling thread to -j$1"
    NJ=$1

else
    NJ=$(nproc --all)
fi
echo "catkin_make -j$NJ"

# # python3 openCV
# catkin_make --pkg vision_opencv -C ./catkin_ws \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#     -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
#     -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
touch ./catkin_ws/src/door_detector_sim/CATKIN_IGNORE

# all
catkin_make -j$NJ -C ./catkin_ws


rm ./catkin_ws/src/door_detector_sim/CATKIN_IGNORE
cd catkin_ws && catkin_make --only-pkg-with-deps door_detector_sim
cd ..

source catkin_ws/devel/setup.bash
