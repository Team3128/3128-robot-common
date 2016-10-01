package org.team3128.common.hardware.encoder.both;

import org.team3128.common.util.units.Angle;

import edu.wpi.first.wpilibj.Counter;

/*        _
 *       / \ 
 *      / _ \
 *     / [ ] \
 *    /  [_]  \
 *   /    _    \
 *  /    (_)    \
 * /_____________\
 * -----------------------------------------------------
 * UNTESTED CODE!
 * This class has never been tried on an actual robot.
 * It may be non or partially functional.
 * Do not make any assumptions as to its behavior!
 * And don't blink.  Not even for a second.
 * -----------------------------------------------------*/


/**
 * Driver for a CTRE Magnetic Encoder using DIO ports on the roborio.
 * 
 * When instantiated, it sets the distance from the absolute angle.
 * So, between 0 and 1 rotations. When reset, the distance goes to zero.
 * 
 * Internally, it uses a Counter to measure the PWM and a WPILib Encoder object
 * to measure the quadrature part.
 * 
 * @author Narwhal
 *
 */
public class CTREMagneticEncoder extends QuadratureEncoder
{
	// had to get this from a forum post by a CTR employee
	public static final int PULSES_PER_REVOLUTION = 1024;

	Counter pwmCounter;

	double m360OffsetDegrees;

	/**
	 * 
	 * @param dataAPort
	 *            DIO port with the "A" data line plugged in (pin 7 on the
	 *            encoder)
	 * @param dataBPort
	 *            DIO port with the "B" data line plugged in to it (pin 5 on the
	 *            encoder)
	 * @param pwmPort
	 *            DIO port connected to pin 9 on the encoder, the PWM pin
	 *            
	 * The PWM signal is used to get absolute position, while the quadrature inputs are used for relative position.
	 * 
	 * @param inverted
	 *            whether or not the encoder is inverted
	 */
	public CTREMagneticEncoder(int dataAPort, int dataBPort, int pwmPort, boolean inverted) 
	{
		super(dataAPort, dataBPort, PULSES_PER_REVOLUTION, inverted);
		
		
		pwmCounter = new Counter(pwmPort);
		pwmCounter.setSemiPeriodMode(false); //only count rising edges
	}

	/**
	 * Returns the absolute angle of the encoder from the PWM signal.  Each full rotation starts this angle back at 0.
	 * @return
	 */
	public double getMod360Angle()
	{
		//from 1 to 4096 us
		return ((pwmCounter.getPeriod() - 1e-6) / 4095e-6) * Angle.ROTATIONS + m360OffsetDegrees;
	}

	@Override
	/**
	 * Resets the absolute angle and the mod 360 angle.
	 */
	public void reset()
	{
		super.reset();
		m360OffsetDegrees += getMod360Angle();
		m360OffsetDegrees = m360OffsetDegrees % 360;
		
	}

}
