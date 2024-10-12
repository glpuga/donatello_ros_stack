#!/bin/bash

# Copyright 2024 Gerardo Puga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <bag directory> <output directory>"
    exit 1
fi

output_dir="output/$1"

mkdir -p $output_dir

echo "Processing $1"
echo "Output will be saved to $output_dir"

ros2 run cartographer_ros cartographer_offline_node \
    -configuration_directory ./configuration_files/ \
    -configuration_basenames narrow_spaces_2d.lua \
    -bag_filenames $1 \
    -save_state_filename $output_dir/posegraph.pbstream

ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -map_filestem $output_dir/map \
    -pbstream_filename $output_dir/posegraph.pbstream
