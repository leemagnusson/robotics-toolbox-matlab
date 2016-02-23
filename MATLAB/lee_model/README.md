Requires robotic-toolbox-matlab from peter corke. Use the version included 
with the git submodules. I.e. run:

$ git submodule update --init

And you should have the requirements.
Then in matlab run:

>> startup_lee.m

in this directory to include the import toolbox files.

From there you can run verb_robot_2015.m and then jinv_control_2015.m for 
a good example of the work in action. verb_robot_2015.m will create a 
SerialLink object in the global workspace named "r". Functions like r.plot(q)
should then work.