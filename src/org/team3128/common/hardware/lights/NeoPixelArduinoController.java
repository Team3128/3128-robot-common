package org.team3128.common.hardware.lights;

import java.util.List;

import org.team3128.common.util.Log;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/** 
 * Controller for the NeoPixel light strip. More accurately,
 * it sends data using digital IO ports on the roboRIO,
 * which is then processed by the Arduino Uno running
 * the corresponding NeoPixelController script.
 * 
 * This class should be extended within each year's robot code
 * in order to have methods defined for specific usage within that
 * years game, such as sustained green blinking or momentary blue blinking.
 * 
 * @author Ronak
 *
 */
public class NeoPixelArduinoController {
	List<DigitalOutput> dataPorts;
	DigitalInput confirmationPort;
	
	/**
	 * Creates a new light controller that is capable of sending single
	 * integer values to the Arduino. Furthermore, you can choose to
	 * maintain a data value or stop sending that data value once the
	 * Uno has confirmed receiving the value.
	 * 
	 * @param confirmationPort - The digital input port at which the Arduino
	 * 		will set to true upon receiving the data if the data sent is
	 * 		defined as non-sustaining.
	 * @param dataPorts - The set of digital output ports that shoudl be used
	 * 		to output data to the Arduino.
	 */
	public NeoPixelArduinoController(DigitalInput confirmationPort, DigitalOutput... dataPorts) {
		for (DigitalOutput dataPort : dataPorts) {
			this.dataPorts.add(dataPort);
		}
		
		this.confirmationPort = confirmationPort;
	}
	
	/**
	 * Clears the data ports and sends over new data. If sustained is true, the data will
	 * simply be set, and it will not be cleared until the ports are manually cleared
	 * or new data is chosen to be sent. If sustained is false, the program will await
	 * the confirmation port, signifying the Arduino has received the data, and then
	 * clear the ports. 
	 * 
	 * @param data - The integer value to be sent to the Arduino.
	 * @param sustained - Whether or not the data should maintain or be erased after being received.
	 */
	public void sendData(int data, boolean sustained) {
		clearPorts();
		char[] portData = Integer.toString(Integer.parseInt(Integer.toString(data), 10), 2).toCharArray();
		
		if (portData.length <= dataPorts.size()) {
			for (int n = 0; n < dataPorts.size(); n++) {
				DigitalOutput port = dataPorts.get(n);
				
				try {
					char bit = portData[n];
					port.set((bit == '0') ? true : false);
				}
				catch (Error e) {
					port.set(false);
				}
			}
			
			if (!sustained) {
				Runnable waitForConfirmation = () -> {
					while (!confirmationPort.get()) {
						try {
							Thread.sleep(50);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
					clearPorts();
				};
				
				new Thread(waitForConfirmation).start();
			}
		}
		else {
			Log.fatal("NeoPixelArduinoController", "Tried to send " + data + " to the Arduino. However, only " + dataPorts.size() + " data ports can be used." );
		}
	}
	
	/**
	 * Sets all of the data ports to zero.
	 */
	public void clearPorts() {
		for (DigitalOutput dataPort : dataPorts) {
			dataPort.set(false);
		}
	}
}
