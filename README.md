# RollingEv3rstorm
Rolling EV3RSTORM: a self balancing robot with a fire gun, built with Lego Mindstorms Ev3

The Rolling EV3RSTORM is a self balancing robot with the following abilities: 
 - it keeps the upright position in equilibrium on its two main wheels; 
 - it moves forward and backward or turns on the spot;
 - it tracks the EV3 IR Beacon with the EV3 IR sensor in its head;
 - it chases the IR Beacon signal;
 - it fires with the gun when at firing range.

Check the video and the building instructions at: http://www.smallrobots.it/balancing-robot/

# Rolling EV3RSTORM functions:
Once you start the robot, it stay still keeping the upright position. If you turn on the remote IR Beacon with key "9", the robot turns the head toward the beacon and follows it; when it reaches the IR Beacon within 10/15 cm, it fires its main gun (no harm at all, but take care of the eyes especially kids'eyes)

# Credits
This robot is a bit complicated, and I couldn’t build it nor program it without some earlier precious work:
- Steven J. Witzland has published a thesis about the GELway: a self-balancing robot built with LEGO and programmed with leJOS (https://code.google.com/archive/p/gelway/)
- Laurens Valk has published on Robot Square the building instructions of the BALANC3R: a self balancing robot from which the Rolling EV3RSTORM inherits a lot; his code for the self-balancing task written with the Lego Mindstorms EV3 Software is also available (http://robotsquare.com/2014/06/23/tutorial-building-balanc3r/).
- Tomoaki Masuda made a valuable work porting the code of Valk in C#. The code is available here on GitHub (https://github.com/moonmile/MonoBalancer).
- The Monobrick firmware library, by Anders and Lars, which makes possibile to program the EV3 Brick in C# (http://www.monobrick.dk/).

