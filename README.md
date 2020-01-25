# LimelightClosestTarget

This is some code that uses the DEEP SPACE 2019 example found on the limelight docs (http://docs.limelightvision.io/en/latest/cs_drive_to_goal_2019.html) and advances it a little bit further.

Essentially, you input a number of wanted distances from the target, and then when you hold a button on your controller, it will automatically move to the closest distance value that you input.

To use it, input your wanted target area / target distance values into the **"kTargetAreas" / "kTargetDistances"** array, then run the code on your bot. Don't forget to tune the constants; there are the proportional loop constants and the max speed values, as well as the stuff for calculating distance. 
If you need help on calculating distance, there's this: http://docs.limelightvision.io/en/latest/cs_estimating_distance.html

Currently, it's setup to move when you hold the A/B button (using targetArea vs targetDistance).

I basically just use 3 for loops, you can read the comments in the code to see how it works. This project was made in a simple TimedRobot project, so I only included the Robot.java file.

Hope this is useful, and if you have any suggestions on how this can be improved, please lmk :)
