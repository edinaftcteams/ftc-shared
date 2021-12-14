# Autonomous Configuration

This class is designed to provide a way for teams to configure autonomous opmodes to perform tasks differently based on information that is learned just before starting a match in competition.

## Some examples:

- Change your navigation path based on your alliance color.
- Select the starting location of you robot.
- Select whether your robot will deliver the duck.
- Select whether your robot will deliver freight to the shipping hub.
- Select the parking location at the end of autonomous.
- Define any delay before starting autonomous.

This configuration class can be modified to meet the requirements of your team and robot.

The "quick-and-dirty" way to manage these options is to create a bunch of different autonomous opmodes using *creative* names to identify their capabilities. 
With the 5 configuration options in this sample class you would need at least 48 different opmodes depending on how you define your options.

*Two big problems with that strategy*:

1. Can the driver reliably pick the correct opmode from the list on the phone in a dark gym with hundreds of cheering fans and extremely loud music blasting from the PA system?
2. When you make a change in your code can you be sure not to make a mistake when adding the change to all of the autonomous opmodes? 

## Code

- AutonomousConfiguration.java - This is the configuration class. Add this to your team code folder and change it to meet your requirements.
- RHSAutonomousMenu.java - This is a basic opmode that uses the configuration class.
__Note:__ *There is a small bonus in this opMode, a state machine example!*
