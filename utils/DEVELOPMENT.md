Changes from Default RMS
===============

Here is a list of the major changes in comparison to the original RMS setup;

* Redesigned Home Page and Navigation in order to allow users immediate access to any active robot interfaces

* Guiabot Interface which is developed for use with our Guiabot Platform which contains an updated and slightly reworked
implementation of the "standard" interfaces from the default RMS, as well as an interface for tracking battery usage.

* Automated Interface which controls functionality for posting information from Internal ROS Nodes to the robot's Twitter 
account.

* Twitter Controller which contains functionality to read messages sent to the robot's twitter account, process them and
if an active hashtag is contained, it will take a photo from the robot's camera and send it to the user in a direct message.

* Reworked Settings Menu to allow for Admins to set a hashtag and it's expiry (default is 3 hours) for use in the controller described above

* Inclusion of Daniel Voyce's CakePHP Twitter Plugin for facilitating Twitter functionality - full information on this plugin can be found [here](https://github.com/voycey/CakePHP-Twitter-API-v1.1-Full).


Twitter Development
===============

* First of all, to use any Twitter functionality you will need to create a Twitter account and then authorise the site as a Twitter app through https://apps.twitter.com/ where you will be provided a secret and public key.

* With a secret and public key, you need to enter them, as well as the account name, in the provided spaces within the bootstrap.php file located [here](/app/Plugin/Twitter/Config/bootstrap.php) as well as in the Twitter_Settings database table.

* There are two locations within the site where Twitter functionality can be added. The first is the automated interface controller located [here](/app/Controller/AutomatedInterfaceController.php) which is where any Twitter developments that do not need to be called every 5 seconds are located.

* If your Twitter functionality needs to be called every 5 seconds (this is the fastest the Twitter API allows you to access status updates), then it can be placed in the Twitter Controller located [here](/app/Controller/TwitterController.php) though some extension work will be required just to allow for multiple functions. You will also have to modify the automated_script.py python script to call your modified functions.

*When your site is running on the server and you want to use the functionality in the Twitter Controller described above, simply run the automated_script.py python code which will call the Twitter controller as close to every 5 seconds as it can.



Areas of Future Development
===============

There are a number of different avenues of future development available for those looking to extend this project;

* Using different social media outlets; instagram (for images) and Vine (for videos) are two good examples of different forms of social media where you could leverage what they do well in order to provide more avenues of user interaction

* Direct robot control; originally an idea for this project was to have site administrators be able to set waypoint navigations for the robot through the interface however this was removed due to scope and is something that would be interesting to see implemented.

* Using the hashtag recognition setup in TwitterController.php in different, interesting ways. Some examples would be commanding the robot to travel to destinations through hashtags, or even implementing a form of hide and seek where the user could tweet at the robot to find them, and it would examine the location metadata provided by Twitter to locate the user.