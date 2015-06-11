<?php

/**
 * Automated Interface View
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
 * @package		app.View.AutomatedInterface
 */
?>

<?php
// setup the main ROS connection and any study information
if($environment['Rosbridge']['host']) {
	echo $this->Rms->ros($environment['Rosbridge']['uri'], $environment['Rosbridge']['rosauth']);
}
echo $this->Rms->initStudy();
?>

<script>
	RMS.logString('start', 'User has connected.');

    
    // Initialize a new ROS Topic that will correspond to the distance_travelled topic being
    // broadcast by the robot

    var distanceTopic = new ROSLIB.Topic({
        ros : _ROS,
        name : '/distance_calculator/distance_travelled',
        messageType : 'distance_calculator/distance_travelled_msg'
    });

    /*  
     *  Subscribe to the distance_travelled topic, create an Ajax data value containing the distance travelled
     *  and the name of the robot, then POST that data to the tweetDistance() php function
     */

    distanceTopic.subscribe(function(message) {

        var data = 'distanceTravelled='+message.distance_travelled.data+'&robotName='+message.robot_name;
        $.ajax({
            url: '<?php echo Router::Url(array('controller' => 'Pages', 'action' => 'tweetDistance'), TRUE); ?>',
            cache: false,
            data: data,
            type: 'POST',
            dataType: 'HTML',
        });

        distanceTopic.unsubscribe();
     });


    // Call the service to reset the distance_travelled count
    var resetDistanceService = new ROSLIB.Service({
        ros: _ROS,
        name : '/distance_calculator/ResetDistanceCount',
        serviceType : 'distance_calculator/ResetCount'
    });

    var request = new ROSLIB.ServiceRequest({
    });

    resetDistanceService.callService(request, function(result) {
    });

    // Initialize a new ROS Topic that will correspond to the people_seen topic being broadcast
    // by the robot

    var peopleSeenTopic = new ROSLIB.Topic({
        ros : _ROS,
        name : '/people_calculator/people_seen',
        messageType : 'people_calculator/people_seen_msg'
    });

    /*  
     *  Subscribe to the people_seen topic, create an Ajax data value containing the number of people seen
     *  and the name of the robot, then POST that data to the tweetPeopleSeen() php function
     */
     peopleSeenTopic.subscribe(function(message) {

        var data = 'peopleSeen='+message.people_seen.data+'&robotName='+message.robot_name;
        $.ajax({
            url: '<?php echo Router::Url(array('controller' => 'Pages', 'action' => 'tweetPeopleSeen'), TRUE); ?>',
            cache: false,
            data: data,
            type: 'POST',
            dataType: 'HTML',
            success: function (data) {
                console.log(data);
            },
        });

        peopleSeenTopic.unsubscribe();
     });


     // Call the Service to reset the count on people_seen
    var resetPeopleService = new ROSLIB.Service({
        ros: _ROS,
        name : '/people_calculator/ResetPeopleCount',
        serviceType : 'people_calculator/ResetCount'
    });


    var request = new ROSLIB.ServiceRequest({
    });

    resetPeopleService.callService(request, function(result) {
    });



 /*         This Code will repeateadly check the group_photo publish, and whenever a photo should be taken it will take one and then tweet it out
            Currently not in use, but is working correctly if uncommented


	var groupPhotoTopic = new ROSLIB.Topic({
        ros : _ROS,
        name : '/group_photo/should_take_photo',
        messageType : 'group_photo/group_photo_msg'
    });


    // Call the Service to reset the count on people_seen
    var resetPhotoTakenService = new ROSLIB.Service({
        ros: _ROS,
        name : '/group_photo/PhotoTaken',
        serviceType : 'group_photo/PhotoTaken'
    });


    var request = new ROSLIB.ServiceRequest({
    });


    groupPhotoTopic.subscribe(function(message) {

        if(message.takePhoto == 0){
            // Do Nothing
            console.log("0");
        }
        else{
            console.log("Photo");

           $.ajax({
                url:'<?php echo Router::Url(array('controller' => 'Pages', 'action' => 'tweetGroupPhotos'), TRUE); ?>',
                cache: false,
                type: 'POST',
            });
            resetPhotoTakenService.callService(request, function(result) {
                 });

        }
     });

*/


    
</script>

<header class="special container">
	<span class="icon fa-cloud"></span>
	<h2>Automated Interface</h2>
</header>

<section class="wrapper style4 container">
	<div class="content center">
		<section>
			<header>
				<p>This page runs an automated service and has no content for end users. Please press the back button in your browser to return to where you were previously, or use the menu above to navigate to a new page.
				</p>
			</header>
</section>
