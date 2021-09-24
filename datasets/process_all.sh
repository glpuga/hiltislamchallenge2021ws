#!/bin/bash

source ../ws/devel/setup.bash

for d in `ls`; do
    if [[ -d $d ]]; then
        pushd $d
        if [[ -f "stamped_groundtruth.txt" ]]; then
            rosrun rpg_trajectory_evaluation analyze_trajectory_single.py . --png 
        else
            echo "Not processing $d because there's no ground truth data"
        fi
    fi
    popd
done
