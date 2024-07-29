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

sudo echo "sudo make me a sandwich"

echo "Removing udev rules..."
pushd rules.d
for file in $(ls); do
    sudo rm /etc/udev/rules.d/$file
done
popd

echo "Restarting udev..."
sudo service udev reload
sudo service udev restart

echo "Done!"
