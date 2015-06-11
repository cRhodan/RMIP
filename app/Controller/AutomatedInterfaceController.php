<?php
App::uses('InterfaceController', 'Controller');

/**
 * Automated Interface Controller
 *
 * This interface is used to automate functionality by calling the page remotely using a cron job
 * Ideally, this page would not be accessible to end user but due to the nature of cakephp, upon
 * which RMS runs, it doesn't appear to be possible to automate the functionality otherwise. As such,
 * the link to this interface will not appear in menus, and will only be accessible by directly accessing
 * the address of the page
 *
 * @author		Chris Rhodan - christopher.rhodan@student.qut.edu.au
 * @copyright	2014 Queensland Univeristy of Technology
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.2
 * @package		app.Controller.AutomatedInterfaceController
 */
class AutomatedInterfaceController extends InterfaceController {

/**
 * The basic view action. All necessary variables are set in the main interface controller.
 *
 * @return null
 */
	public function view() {
		// set the title of the HTML page
		$this->set('title_for_layout', 'Basic Interface');
		// we will need some RWT libraries
		$this->set('rwt', array('roslibjs' => 'current', 'keyboardteleopjs' => 'current'));


	/**
*	Posts a tweet to twitter dictating the distance travelled by the robot over the current time period and
*	then signs off with the name of the robot as given through the distance_travelled_msg message.
*	@param float $distanceTravelled The distance that has been travelled (in meters) by the Robot
*	@param string $robotName The name of the corresponding robot
*	@return null
*
*/

	public function tweetDistance(){
		// Retrieve the distanceTravelled and robotName variables through POST
		// as this function should only be called using AJAX
		$distanceTravelled = $_POST['distanceTravelled'];
		$robotName = $_POST['robotName'];

		// Retrieve the list of possible message bodies and select one at random
		$messages = $this->Twitter->findAllByContext("Distance"); 
		$messageIndex = rand(0, count($messages) - 1);
		$message = $messages[$messageIndex]['Twitter']['content'];


		// Replace the placeholder Robot Name and Distance Travelled
		$message = str_replace('#CONTENT#', $distanceTravelled, $message );
        $message = str_replace('#ROBOT_NAME#', $robotName, $message); 

		// Load the Twitter functionality through the Twitter model
		$this->loadModel('Twitter.Twitter');

		// Convert the distance to a float with three decimal places
		$convertedDistance = number_format($distanceTravelled, 3, '.', '');

		// Post the tweet to twitter
		$twitter = $this->Twitter->post('statuses/update',array('status'=>$message));
	}


/**
*	Posts a tweet to twitter dictating the number of people seen by the robot over the current time period and
*	then signs off with the name of the robot as given through the ROS message.
*	@param float $peopleSeen The number of People seen by the Robot
*	@param string $robotName The name of the corresponding robot
*	@return null
*
*/

	public function tweetPeopleSeen(){
		// Retrieve the number of people seen and robotName variables through POST
		// as this function should only be called using AJAX
		$peopleSeen = $_POST['peopleSeen'];
		$robotName = $_POST['robotName'];

		// Retrieve the list of possible message bodies and select one at random
		$messages = $this->Twitter->findAllByContext("PeopleSeen"); 
		$messageIndex = rand(0, count($messages) - 1);
		$message = $messages[$messageIndex]['Twitter']['content'];

		// Replace the placeholder Robot Name and Distance Travelled
		$message = str_replace('#CONTENT#', $peopleSeen, $message );
        $message = str_replace('#ROBOT_NAME#', $robotName, $message); 

		// Load the Twitter functionality through the Twitter model
		$this->loadModel('Twitter.Twitter');


		// Post the tweet to twitter
		$twitter = $this->Twitter->post('statuses/update',array('status'=>$message));
	}

}
