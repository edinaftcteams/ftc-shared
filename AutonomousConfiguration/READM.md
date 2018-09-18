# Autonomous Configuration
This class is designed to provide a way for teams to configure autonomous opmodes to perform tasks differently based on information that is learned just before starting a match in competition.
## Some examples:
- Change your navigation path based on your alliance color.
- Change your navigation path based on where you start on the *Lander* (latched or not.)
- Delay starting to prevent interfering with your alliance partner's autonomous tasks.
- Enable or disable scoring tasks to customize your strategy.

As the season evolves you will undoubtedly discover many more.

The "quick-and-dirty" way to manage these options is to create a bunch of different autonomous opmodes using *creative* names to identify their capabilities. 
With the 4 configuration options above, and assuming your delay start option is either delay n seconds or no delay, and you only have one optional task, you would need 16 different opmodes.
That still would not allow you to vary the start delay time!

2 big problems with that strategy:
1. Can the driver reliably pick the correct opmode from the list on the phone in a dark gym with hundreds of cheering fans and extremely loud music blasting from the PA system?
2. When you make a change in your code can you be sure not to make a mistake when adding the change to the rest of your autonomous opmodes? 

## Code
- AutonomousConfiguration.java - This is the configuration class. Please change it to meet your requirements.
- CakePushbotAutoDriveByEncoder_Linear.java - This is a sample opmode that uses the configuration class.