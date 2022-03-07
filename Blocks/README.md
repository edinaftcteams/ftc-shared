# Blocks

Blocks is a great way to get code running quickly.

## Source Control

One problem with Blocks has been protecting your code using source control. Hopefully this will be addressed by FIRST in the future. In the mean time the following procedure will keep you from experiencing that "Oh no..." moment.


**You must backup your code!**

Code stored in only 1 location (Rev Control Hub, laptop, Chromebook, flash drive, etc.) is not enough! You can learn this lesson the hard way when your control hub dies during inspection at you first competition, or define your backup plan now and stick with it.

In Android Studio, GitHub is the recommended tool for protecting your code. Unfortunately it is a bit complicated to make Blocks, OnBot and GitHub work together. NOTE: If you find a simple solution for Blocks, OnBot and GitHub please share it.

## Blocks and OnBot Backup

Blocks and OnBot now have a *Backup / Restore* tab with a *Backup Files* button which creates a zip file that contains all .xml configuration files, Blocks .blk and .js files and OnBotJava .java files and copies it to your computer. The file name will be displayed next to the *Backup Files* button after it is creatred. Clicking on this file name you will go to the file's location on your computer.

To restore files to the Rev Control Hub use the *Choose Files to Restore* button on the *Backup / Restore* tab.

### Recommended Backup Process

1. After every coding session, or more often if you make a lot of changes, backup your work using the *Backup Files* button.
2. Connect a second computer to your control hub and use the *Backup Files* button to create a backup on that computer. You now have the code backed up on 2 separate devices.
3. If you don't have a second cumputer available, sopy the backup zip file to a flash drive or somewhere on the cloud.
4. **Do this every time you edit code, all season long!**

This may seem a little cumbersome, but until GitHub is integrated into Blocks and OnBot, it is better than losing you code.

# Offline Edit

Usually Blocks programming is done while connected to the robot controller. The disadvantage to this is the need to be connected for programming. Blocks has a built-in editor for offline programming. Here are the steps for installing it.

1. Connect to your controller from your laptop browser: 192.168.43.1:8080
2. Click on "Program and Manage".
3. Click on "Blocks".
4. Click on "Download Offline Blocks Editor". This will download an zip file to your download folder.
5. Unzip the downloaded file into a new folder.
6. To run the offline editor, click on the index.html file in your new folder.
7. Create or edit your Op Modes.
8. To upload the Op Mode to the controller, select the Op Mode(s) and click "Download Selected Op Modes". This will create .blk files in your download folder.
9. Now reconnect to your robot controller (step 1 above) and click "UpLoad Op mode". Click on "Choose File" and select the .blk file in your download folder. Click "Ok". The Op Mode will now be on your controller.

**Important Notes:**

* The zip created when you click "Download Offline Blocks Editor" will be created with a name similar to *configfile*_offline_blocks_editor.zip where *configfile* is the name of the active config file. If you have multiple config files, then you will want to create multiple offline editors to match you config files. Just put each editor in its own folder. You might want to create a catch all config file that contains all the devices you might want to use for testing or learning. Use the Group menu option to keep them organized in the driver station drop down.

* Multiple users can connect to the controller at the same time, just edit using unique op mode names. Using this takes some cooperation to make sure you don't mix up code.
