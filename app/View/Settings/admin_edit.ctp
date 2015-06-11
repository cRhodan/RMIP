<?php
/**
 * Admin Settings Edit View
 *
 * The settings edit page allows an admin to edit site settings.
 *
 * @author		Russell Toris - rctoris@wpi.edu
 * @copyright	2014 Worcester Polytechnic Institute
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.2
 * @package		app.View.Settings
 */
?>

<header class="special container">
	<span class="icon fa-edit"></span>
	<h2>Edit Site Settings</h2>
</header>

<section class="wrapper style4 container">
	<div class="content">
		<section>
			<header>
				<h3>Enter New Site Settings Below</h3>
			</header>
			<?php
			echo $this->Form->create('Setting');
			echo $this->Form->input('id', array('type' => 'hidden'));
			?>
			<div class="row">
				<section class="6u">
					<?php echo $this->Form->input('title'); ?>
					<br />
					<?php echo $this->Form->input('analytics', array('label' => 'Google Analytics ID (optional)')); ?>
				</section>
				<section class="6u">
					<?php echo $this->Form->input('hashtag', array('label' => 'Twitter Photo Hashtag')); ?>
					<br />
					<label>Hashtag Expiry</label> <br/>
					<?php echo $this->Form->dateTime('hashtagExpiry', 'DMY', '24', array('label' => 'Hashtag Expiry', 
													'value' => array(
																    'day' => Date('j'),
																    'month' => Date('F'),
																    'year' => Date('Y'),
																    'hour' => Date('G') + 3,
																    'min' => Date('i'),
																    ))); ?>
					<p style="font-size: small; color:red">Please remember that anyone who knows the hashtag will be able to take a photo through the robot's camera until this expiry time. It is reccomended that a hashtag only last until the end of an event the robot is attending. The default expiry time is three hours from when the hashtag is changed. </p>

				</section>
			</div>
			<div class="row">
				<section class="12u">
					<?php echo $this->Form->end(array('label' => 'Save', 'class' => 'button special')); ?>
				</section>
			</div>
		</section>
	</div>
</section>
