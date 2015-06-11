<?php
/**
 * Default Layout
 *
 * The main, default layout uses the RMS template to create a new page with a menu, content section, and footer.
 *
 * @author		Russell Toris - rctoris@wpi.edu
 * @copyright	2014 Worcester Polytechnic Institute
 * @link		https://github.com/WPI-RAIL/rms
 * @since		RMS v 2.0.0
 * @version		2.0.2
 * @package		app.View.Layouts
 */
?>

<?php $home = isset($home) && $home; ?>

<!DOCTYPE html>
<html>
<?php echo $this->element('head'); ?>
<?php echo $this->Html->script('jquery.dropotron.min'); // Include jQuery library ?>

<script >
	$(window).load(function(){
		
		/* Once the window loads, there will be either a span#thumbdown or span#thumbup element on each robot's interface entry to indicate whether rosbridge and/or mjpeg servers are up or down. For each interface, if it's a thumbdown (i.e. it's inactive) then the entry added to the inactive heading content area, and if its a thumbup then it is applied to the active heading content area */

		// TODO: rather than chained parents, this should probably be an additional selector

		$('span#thumbdown').each(function(){
			$(this).parent().parent().parent().parent().appendTo("#inactive");
		});

		$('span#thumbup').each(function(){
			$(this).parent().parent().parent().parent().appendTo("#active");
		});


		// These two JQuery functions toggle the Active/Inactive content areas when the respective title is clicked

		$('#accordion-content-active').click(function(){
			$("#active").toggle();
			$('img', this).toggle();
		});

		$('#accordion-content-inactive').click(function(){
			$("#inactive").toggle();
			$('img', this).toggle();
		});

	});
</script>


<body class="<?php echo ($home) ? 'index' : 'no-sidebar'; ?> loading">
	<div id="menu">
		<?php echo $this->element('menu', array('home' => $home)); ?>
	</div>


	<div id="content-wrapper">

		
		
		<?php

			if ($home) { ?>
				<?php if(count($ifaces) > 0): ?>
				<hr />
				<section id="interfaces" class="wrapper style4 container">
					<div class="content center">
						<header>
							<h2> Welcome to the Robots @ QUT Page</h2>
							<p>Please select a Robot from the list below</p>
						</header>
					</br>
					</div>

					<div id="accordion-content-active">
						<h2 id="active-header" class="robot-dropdown"> Active Robots </h2>
						<img id="arrow-icon" src="img/down_arrow.png"></img>
						<img id="arrow-icon" src="img/right_arrow.png" style="display:none"></img>
					</div>
					<br />
					<div class="content center" id="active">
						<?php foreach ($ifaces as $iface): ?>
						<?php if($iface['Iface']['name'] != "Automated"): ?>
						<div id="interface-row">
							<div class="row center">

								<section class="12u">
									<strong><u><?php echo h($iface['Iface']['name']); ?></u></strong>
								</section>

								<?php foreach ($iface['Environment'] as $environment): ?>

									<section class="4u">
										<?php
										echo $this->Html->link(
											$environment['name'],
												array(
													'controller' => 'ifaces',
													'action' => 'view',
													$iface['Iface']['id'],
													$environment['id']
												)
											);
										?>
									</section>

									<section class="4u">
										<strong>rosbridge Status:</strong>
										<?php if (!isset($environment['Rosbridge']['id'])): ?>
											N/A
										<?php else: ?>
											<?php
											echo $this->Rms->rosbridgeStatus(
												$environment['Rosbridge']['Protocol']['name'],
												$environment['Rosbridge']['host'],
												$environment['Rosbridge']['port']
											);
											?>
										<?php endif; ?>
									</section>

									<section class="4u">
										<strong>MJPEG Status:</strong>
										<?php if (!isset($environment['Mjpeg']['id'])): ?>
											N/A
										<?php else: ?>
											<?php
											echo $this->Rms->mjpegServerStatus(
												$environment['Mjpeg']['host'],
												$environment['Mjpeg']['port']
											);
											?>
											<br />
										<?php endif; ?>
									</section>

								<?php endforeach; ?>
								
							</div>
							<hr />
						</div>
						<?php endif; ?>
						<?php endforeach; ?>
					</div>

					<br />

					<div id="accordion-content-inactive">
						<h2 id="inactive-header" class="robot-dropdown"> Inactive Robots </h2>
						<img id="arrow-icon" src="img/down_arrow.png" style="display:none"></img>
						<img id="arrow-icon" src="img/right_arrow.png"></img>
					</div>
					<br />
					<div class="content center" id="inactive" style="display: none;">	
					</div>
				</section>


				
			<?php endif; ?>
		<?php } ?>

		

		<?php if (!$home) { ?>
			<article id="main">
				<?php if($this->Session->check('Message.flash')): ?>
					<section class="flash">
						<p><?php echo $this->Session->flash(); ?></p>
					</section>
				<?php endif; ?>
				<?php echo $this->fetch('content'); ?>
			</article>
		<?php } ?>


		<div class="push"></div>
	</div>

	<?php echo $this->element('footer'); ?>
</body>




</html>


