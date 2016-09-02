package org.team3128.common.listener.controllers;

import org.team3128.common.listener.controltypes.Axis;
import org.team3128.common.listener.controltypes.Button;

/**
 * Controller object for a Logitech Extreme3D joystick.
 * 
 * NOTE: I've omitted all of the numbered buttons. You can just use the new Button constructor.
 * @author Jamie
 */
public class ControllerExtreme3D
{
		public static final Button TRIGGER = new Button(1);
		
		public static final Axis JOYX = new Axis(0);
		public static final Axis JOYY = new Axis(1);
		public static final Axis TWIST = new Axis(2);
		public static final Axis THROTTLE = new Axis(3);

		
		private ControllerExtreme3D()
		{
			
		}
}
