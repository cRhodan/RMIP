<?php
/**
 * Guiabot Interface View
 *
 * The Guiabot interface displays a 2D gridmap, robot pose, camera feed and keyboard teleop.
 * A simple ros topic publisher and subscriber are also included for testing purpose.
 *
 * @author		Inkyu Sa, enddl22@gmail.com
 * @copyright		2014, QUT, cyphylab.
 * @version		0.0.1
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
</script>



<?php 
    // This is how to retrieve the current tweet info
    // echo $current_tweet 
?>


<section>

    <div id="battery-div">

        <script>
            // TODO: Read battery max/min from somewhere
            var batteryMax = 29;
            var batteryMin = 24;
            var batteryDiff = batteryMax - batteryMin;
            var currentValue;

            // Initialize a new ROS Topic that will read the current state of the battery

            var batteryTopic = new ROSLIB.Topic({
                ros : _ROS,
                name : '/battery_state',
                messageType : 'p2os_driver/BatteryState'
            });

            /*  
             *  Subscribe to the battery topic, read the current value and then unsubscribe
             */

              console.log("test");

            batteryTopic.subscribe(function(message) {
                currentValue = message.voltage;
               

                batteryTopic.unsubscribe();
            });


            // JQuery (and some JS) to format the battery meter

            $(window).load(function(){
                $(this).css('background-color', ''); // Clear the background colour

                // If the currentValue is undefined, then it can't connect to ROS (or something else has gone wrong)
                // so we will just display an error message in that case

                if(typeof currentValue !== 'undefined'){
                    if(currentValue > (batteryMin + (0.66 * batteryDiff) )){
                        // Essentially full, use green to fill
                        $('#battery-div').css('background-color', 'green');
                    }
                    else if(currentValue > (batteryMin + (0.33 * batteryDiff) )){
                        // Intermediate, use yellow
                        $('#battery-div').css('background-color', 'yellow');
                    }
                    else{
                        // Nearly empty, use Red
                        $('#battery-div').css('background-color', 'red');
                    }

                     // Calculate current percentage to 2 decimal place
                    var percentage = ((currentValue - batteryMin) / batteryDiff * 100).toFixed(2);

                    $('#battery-percentage').text("Current Battery: " + percentage + "%");
                    $('#battery-div').css('width', percentage + "%");
                }

                else{

                    // If the value is undefined, then display a message to the user saying battery data is unavailable
                    $('#battery-percentage').text("Battery Information is Currently Unavailable");

                }

               // Jquery to support the show/hide of screen elements 

               $('.streamElementToggler').click(function(){
                    $("#showStreamSection").toggle();
                    $("#streamSection").toggle();
                    $('.streamElementArrow').toggle();
                });

                $('.twistElementToggler').click(function(){
                    $("#showTwistSection").toggle();
                    $("#twistSection").toggle();
                    $('.twistElementArrow').toggle();
                });

                $('.mapElementToggler').click(function(){
                    $("#showMapSection").toggle();
                    $("#mapSection").toggle();
                    $('.mapElementArrow').toggle();
                });


            });                  
        </script>
        <p id='battery-percentage' align='center'>100%</p>
    </div>

    <div id="column1-wrap">
        <div id="">
            <section id="showStreamSection" style="display:none; float:left" class="streamElementToggler">
                <img id="arrow-icon" class="streamElementArrow showHideArrow" src="../../img/right_arrow.png" style="display:none" title="Show Video Stream"></img>
            </section>
            <section id="streamSection" class="12u stream" style="float:left; width:49%;">
                <?php if($environment['Mjpeg']['host']): ?>
                <div id= "showHideDiv" class="streamElementToggler">
                    <img id="arrow-icon" class="streamElementArrow showHideArrow" src="../../img/down_arrow.png" title="Hide Video Stream"></img>
                </div>
                <div id="streamContentDiv">
                    <?php if(count($environment['Stream']) > 0): ?>
                        <?php
                        echo $this->Rms->mjpegStream(
                            $environment['Mjpeg']['host'],
                            $environment['Mjpeg']['port'],
                            $environment['Stream'][0]['topic'],
                            $environment['Stream'][0]
                        );
                        ?>
                    <?php else: ?>
                        <h2>No Associated MJPEG Streams Found</h2>
                    <?php endif; ?>
                <?php else: ?>
                    <h2>No Associated MJPEG Server Found</h2>
                </div>
                <?php endif; ?>
            </section>
            
            <div>
                 <section id="showMapSection" style="display:none; float:left" class="mapElementToggler" style="margin-left:50%">
                    <img id="arrow-icon" class="mapElementArrow showHideArrow" src="../../img/right_arrow.png" style="display:none" title="Show Map"></img>
                </section>
                <section id="mapSection">
                    <div id= "showHideDiv" class="mapElementToggler">
                        <img id="arrow-icon" class="mapElementArrow showHideArrow" src="../../img/down_arrow.png" title="Hide Map"></img>
                    </div>
                    <?php echo $this->Rms->ros2d('#ffffff'); ?>
                </section>
            </div>

             <section id="showTwistSection" style="display:none; float:left" class="twistElementToggler">
                <img id="arrow-icon" class="twistElementArrow showHideArrow" src="../../img/right_arrow.png" style="display:none" title="Show Movement Data"></img>
            </section>
            <section id="twistSection" class="12u stream"  style="float:left; width:49%;">
                <header>
                    <p><strong>/cmd_vel</strong> ros Twist Message</p>
                </header>
                <div id= "showHideDiv" class="twistElementToggler">
                    <img id="arrow-icon" class="twistElementArrow showHideArrow" src="../../img/down_arrow.png" title="Hide Movement Data"></img>
                </div>
                <br/> <br/>
                <?php if($environment['Rosbridge']['host']): ?>
                    <?php if(count($environment['Teleop']) > 0): ?>
                        <?php echo $this->Rms->keyboardTeleop($environment['Teleop'][0]['topic']); ?>
                        <pre class="rostopic"><code id="speed">Awaiting data...</code></pre>
                        <script>
                            var topic = new ROSLIB.Topic({
                                ros : _ROS,
                                name : '<?php echo h($environment['Teleop'][0]['topic']);?>'
                            });
                            topic.subscribe(function(message) {
                                $('#speed').html(RMS.prettyJson(message));
                            });
                        </script>
                    <?php else: ?>
                        <h2>No Associated Telop Settings Found</h2>
                    <?php endif; ?>
                <?php else: ?>
                    <h2>No Associated rosbridge Server Found</h2>
                <?php endif; ?>
            </section>
        </div>
    </div>
    
    <div id="clear"></div>
</section>

<script>
    // Setup the nav client.
    new NAV2D.OccupancyGridClientNav({
        ros : _ROS,
        rootObject : _VIEWER2D.scene,
        viewer : _VIEWER2D,
        topic : '/web_map',
        serverName : '/move_base',
        withOrientation : true
    });
</script>






