<?php
/**
 * 	A simple controller for handling Twitter functionality
 *	Since it should solely be called by the server, we just display a page containing an error message if the
 *	page is accessed.
 */
class TwitterController extends AppController {

	/**
	 * The used helpers for the controller.
	 *
	 * @var array
	 */
	public $helpers = array('Html', 'Form', 'Time', 'Rms');


	/**
	 * The used models for the controller.
	 *
	 * @var array
	 */
	public $uses = array('Iface');

	/**
	 * The used components for the controller.
	 *
	 * @var array
	 */
	public $components = array('Session', 'RequestHandler');

	/**
	 * Define the actions which can be used by any user, authorized or not.
	 *
	 * @return null
	 */
	public function beforeFilter() {
		// only allow unauthenticated viewing of a single page
		parent::beforeFilter();
		$this->Auth->allow();
	}


	/**
	 * The default index simply redirects to the homepage action.
	 *
	 * @return null
	 */
	public function index() {
		return $this->redirect(array('action' => 'view'));
	}


	/**
	* Display an invalid page error message if the page is directly accessed by a user
	*/
	public function view($id = null) {

		throw new NotFoundException('Invalid page.');
		
	}


	/*
		Retrieves all messages sent to the robot on Twitter and processes them to check if they contain the
		hashtag specified by the admin. Then, any matches that have not previously been processed by the
		application are processed and the image sent to the user
	*/


	public function retrieveMentions(){

		//$hashtagToFind = '#photo';

		$this->loadModel('Twitter.Twitter');


		/* Doing it this method, means we can only make 15 requests every 15 minutes as per twitter API, this is not frequent enough

			$results = $this->Twitter->get('statuses/mentions_timeline', array('count'=>'10'));
		*/

		$setting = $this->Setting->findById(Setting::$default);
		$hashtag = $setting['Setting']['hashtag'];

		// If the hashtag has expired, Post a tweet about it and then return
		$currentTime = new DateTime();
		$hashtagExpiry = date_create_from_format('j-m-Y-G-i', $setting['Setting']['hashtagExpiry']);

		if($hashtagExpiry < $currentTime){
			// Tweet out that the Hashtag has expired
			$status = "The hashtag ".$hashtag." has now expired. Thanks - Guiabot";
			$twitter = $this->Twitter->post('statuses/update', array('status'=>$status));

			return false;
		}

		$lastTweetPhoto = $setting['Setting']['lastTweetPhoto'];
		$lastTweetDate = $setting['Setting']['lastTweetDate'];
		

		// We need to convert the hashtag into the correct format for Twitter processing
		// %23 creates the hash and %20%40 creates the @link to the qutRobot account
		$hashtagFormatted = "%23".$hashtag.'%20%40qutRobot';

		// By retrieving all tweets with the specified hashtag, and taking the most recent one this way, we can make 180 calls per 15 min
		// We also bypass the need to check if the hashtag is in any of the tweets ourselves
		$tweets = $this->Twitter->get('search/tweets', array('q'=>$hashtagFormatted, 'include_entities'=>'true'));
		$results = $tweets->statuses;


		// Check if this is the same tweet as the last one to have it's photo taken
		// and if it is, do nothing

		for($i = 0; $i < count($results); $i++){
			if ($results[$i]->id == $lastTweetPhoto){
				return;
			}

			// Convert the created_at value given by twitter (which is of the form Tue Apr 21 23:43:22) to a timestamp
			$datetime = new DateTime($results[$i]->created_at);
			$datetime = $datetime->format('U');

			if($datetime < $lastTweetDate){
				// Older tweet than the last one a photo was taken for, so ignore it
				// (This situation occurs if a tweet is processed by the application, then the user deletes the tweet after a photo 
				// has been taken - the application would otherwise think the older photo is a new one and take a photo)
				return;
			}

			else{

				$lastTweetDate = $datetime;

				// Take a photo
				$photo = file_get_contents('http://localhost:8080/snapshot?topic=/stereo/left/image_raw');

				// Generate a random md5 hash to store as file name (so people cannot simply guess the name of a photo)
				$photoName = md5(uniqid(rand(), true));

				// Save the file to the given directory
				file_put_contents('/home/rhodan/website/rms/app/webroot/img/group_photos/'.$photoName.'.jpg', $photo);

				$lastTweetPhoto = $results[$i]->id;

				$result = $this->Setting->updateAll( array('lastTweetPhoto' => $results[$i]->id));
				$result2 = $this->Setting->updateAll( array('lastTweetDate' => $lastTweetDate));

				// Send a link to the image to the user who requested it via Direct Message
				// Note they can only actually receive the DM if they are following the robot
				
				$status = "Your photo can be found at: "."http://localhost/webroot/img/group_photos/".$photoName.".jpg";
				$userID = $results[$i]->user->screen_name;

				$twitter = $this->Twitter->post('direct_messages/new',array('text'=>$status, 'screen_name'=>$userID));
				
				return;
			}
		}		
	}


}

?>