#### RMIP (Robot Management and Interaction Platform)

This RMIP project extends the Robot Management System (RMS) developed by Russell Toris et al. RMS is a remote lab and user study management tool designed for use with controlling ROS enabled robots from the web. RMS is built on-top of the popular [CakePHP](http://cakephp.org/) Model-View-Controller (MVC) framework.

For full documentation on the RMS, see [the ROS wiki](http://ros.org/wiki/rms) or check out some [tutorials](http://www.ros.org/wiki/rms/#Tutorials).

This project has developed two custom interfaces; one (Guiabot Interface) for controlling a robot platform running the [Robot Operating Systems](http://wiki.ros.org/) and the other (Automated Interface) providing an avenue for automating a connection between the robot running ROS and Twitter. This platform has been tested using a guiabot platform in operation at Queensland University of Technology, Brisbane but has been developed so that it can be used by any robot running ROS.

This project creates it's own custom interfaces (Guiabot Interface and AutomatedInterface) for the purpose of using a robot in operation at Queensland University of Technology, Brisbane, Australia in order to interact with a wider public.

It does this through two primary avenues of communciation; the web interface for interacting with the robot directly (Guiabot Interface), and through the Twitter social media platform controlled via the Automated Interface.

To facilitate this, two simple ROS Nodes were also created and these are included in the [folder here](/ROS_Nodes).

### Setup

To install the RMS base setup, please follow the steps found on the RMS repository [Here](https://github.com/WPI-RAIL/rms). Once this setup is complete, then create a backup of the database.php file found at: "rms/app/config/database.php" and replace the RMS folder with the one contained in this repository, replacing the databse.php file from this repository with your own.

To use the functionality provided by the site, you will need to change a couple of addresses to match your installation. The lines that need to be changed are lines 135, 141 and 151 in [TwitterController.php](/app/Controller/TwitterController.php) and line 13 in [automated_script.py](automated_script.py). Change these to reflect the file path in your server.

### Developers Guide

For those looking to see what has changed in this project compared to the vanilla RMS setup, and/or are thinking about extending the work performed, please check out our [development guide](utils/DEVELOPMENT.md).

### Build
Checkout [utils/README.md](utils/README.md) for details on building if you are contributing code.

### License
RMS is released with a BSD license. For full terms and conditions, see the [LICENSE](LICENSE) file.
