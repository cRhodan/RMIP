<?php
App::uses('InterfaceController', 'Controller');

class GuiabotInterfaceController extends InterfaceController {

        public function view() {
                // set the title of the HTML page
                $this->set('title_for_layout', 'Guiabot Interface');
                // we will need some RWT libraries
                $this->set('rwt', array('roslibjs' => 'current' ,
                                        'ros2djs' => 'current',
                                        'nav2djs' => 'current',
                                        'ros3djs'=>'current',
                                        'keyboardteleopjs' => 'current'));

        }

        
}
