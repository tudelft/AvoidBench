ADC and GPIO Library
====================

This library is tested with Hardkernel Odroid boards and Intel Up boards and is released under a MIT license. 
It was developed at the [Robotics and Perception Group](http://www.ifi.uzh.ch/en/rpg.html) to be used within our [quadrotor framework](https://github.com/uzh-rpg/rpg_quadrotor_control).

Attention!!!
------------

Odroids use 1.8V for all teir IO signals, so make sure that you use a level shifter (e.g. [this one](https://www.digikey.ch/product-detail/de/sparkfun-electronics/BOB-11771/1568-1208-ND/5673794)) when using GPIOs with external signals of different voltage levels and use an appropriate [voltage divider](https://en.wikipedia.org/wiki/Voltage_divider) when using an ADC port!

Intel Up boards have the signals on their pin header shifted to 3.3V. Also make sure not to exceed these values with external voltages.

Use with Odroid XU3/XU4
-----------------------
For GPIO pins to be used, they need to be exported with `echo $PIN_NUMBER > /sys/class/gpio/export`. This is taken care of by rpg_single_board_io. However, the newly exported GPIO pins have their permissions set to root-access only. We can change that by adding
corresponding udev rules:

First create `/etc/udev/rules.d/10-gpio.rules`, for instance by using `touch`.

Then add the following two lines to `/etc/udev/rules.d/10-gpio.rules`:

```
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys/class/gpio/export /sys/class/gpio/unexport ; chmod 220 /sys/class/gpio/export /sys/class/gpio/unexport'"
SUBSYSTEM=="gpio", KERNEL=="gpio*", ACTION=="add", PROGRAM="/bin/sh -c 'chown root:odroid /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value ; chmod 660 /sys%p/active_low /sys%p/direction /sys%p/edge /sys%p/value'"
```

[source](http://forum.odroid.com/viewtopic.php?f=80&t=15000)

For the changes to take effect, either reboot the odroid or run

`udevadm control --reload-rules`

or simply reboot the odroid. 


Use with Odroid U3
------------------
TODO

Use with Up board
-----------------
On the Up board, there are only a few points you need to check if you have issues with permissions (symptom is usually an error `Could not set necessary configuration of serial port` from the [SBUS bridge](https://github.com/uzh-rpg/rpg_quadrotor_control/tree/master/bridges/sbus_bridge)

1.  You are using an "upboard" kernel. Check for that using `uname -a` and look for any mention of "upboard" in the kernel's name.
      -  If not, go to this page and install such a kernel: [https://wiki.up-community.org/Ubuntu](https://wiki.up-community.org/Ubuntu)
   
2.  You have set the permissions and installed the add-ons as described here: [https://wiki.up-community.org/Ubuntu#Enable_the_HAT_functionality_from_userspace](https://wiki.up-community.org/Ubuntu#Enable_the_HAT_functionality_from_userspace)

3.  You might be using the wrong port name. The correct port name is usually `/dev/ttyS1` or `/dev/ttyS4`.
     -   Make sure that you don't overwrite the parameters you set. For the SBUS bridge for instance, the value is set in the parameters `.yaml` file but it could be overwritten in another parameter file or launch file. Thus, check the ROS info message when you launch the node. Note that `/dev/ttyS0` is reserved for the console, so enter `cat /dev/ttySX` and try the ports which do not output any error (don't use `sudo`).


Alternative
-----------

To give a single executable root privileges you can do the following steps:

```
cd catkin_ws/devel/lib/YOUR_PACKAGE
sudo chown root:root YOUR_EXECUTABLE && sudo chmod a+rx YOUR_EXECUTABLE && sudo chmod u+s YOUR_EXECUTABLE
```

**Note:** You have to redo this step each time you recompile the executable
