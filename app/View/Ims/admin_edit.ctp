<?php
/**
 * Admin Edit Marker View
 *
 * The edit marker view allows an admin to edit an existing marker setting in the database.
 *
 * @author		Russell Toris - rctoris@wpi.edu
 * @copyright	2014 Worcester Polytechnic Institute
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.2
 * @package		app.View.Ims
 */
?>

<header class="special container">
	<span class="icon fa-pencil"></span>
	<h2>Edit Interactive Marker Setting</h2>
</header>

<?php echo $this->element('im_form', array('edit' => true)); ?>
