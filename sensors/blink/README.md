# Blink(1) ROS node #

### What is this repository for? ###

This node provides a ROS service and a ROS topic that manipulates [Blink(1)](https://blink1.thingm.com/). Accompanies the node a service API that facilitates the use of the service. This API is not ideal for real time use. For real time, use the topic version.

### How do I get set up? ###

The code depends on libUSB. To install libUSB:


```
#!bash

sudo apt-get install libusb-dev

```

To install the Blink1 package:

Create a catkin workspace. For instructions on how to create the workspace go [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
Download and compile the package:

```
#!bash

cd catkin_ws/src
git clone git@bitbucket.org:castacks/blink1_node.git
cd ..
catkin_make
```

To run and test the node you must have permission to access the Blink(1) device. For a first test you may change to root (`sudo -s`) and run:

```
#!bash

source devel/setup.bash
roslaunch blink1 blink1.launch
```

In another terminal:

```
#!bash

source devel/setup.bash
rosrun blink1 blink1_example
```

or 

```
#!bash

source devel/setup.bash
rosrun blink1 blink1_example_topic
```


The led should turn on and blink with several colors. See the examples folder to the code of these examples.

To permanently change the permissions of the device and run the node in the user mode, please refer to the [instructions of the device's manufacturer](https://bitbucket.org/castacks/blink1_node/src/master/examples/blink.rules).

### Who do I talk to? ###

* Guilherme Pereira (gpereira@ufmg.br)

### License ###
[This software is BSD licensed.](http://opensource.org/licenses/BSD-3-Clause)

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
