<?php
/**
 * Default Footer
 *
 * The footer contains a copyright note and acknowledgement to the RMS.
 *
 * @author		Russell Toris - rctoris@wpi.edu
 * @copyright	2014 Worcester Polytechnic Institute
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.2
 * @package		app.View.Elements
 */
?>

<footer id="footer">
	<span class="copyright qut">
		<a href="http://www.qut.edu.au">
			&copy; <?php echo __('%s %s', date('Y'), h($setting['Setting']['copyright'])); ?>
		</a>
	</span>

	<span class="copyright rms">
		<a href="http://wiki.ros.org/rms">
			Powered by the	
			<strong>Robot Management System</strong>
		</a>
	</span>
</footer>
