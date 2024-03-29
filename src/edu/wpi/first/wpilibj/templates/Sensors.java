/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Driver
 */
import edu.wpi.first.wpilibj.*;

public class Sensors extends SensorBase {

    //private final double IN_TO_CM_CONVERSION = 2.54;
    private boolean use_units;    //Are we using units or just returning voltage?
    //private double min_voltage;	  //Minimum voltage the ultrasonic sensor can return
    //private double voltage_range; //The range of the voltages returned by the sensor (maximum - minimum)
    //private double min_distance;  //Minimum distance the ultrasonic sensor can return in inches
    //private double distance_range;//The range of the distances returned by this class in inches (maximum - minimum)
    AnalogChannel channel;
    //constructor
    public Sensors(int _channel) {
        channel = new AnalogChannel(_channel);
        //default values
		use_units = true;
	//	min_voltage = .5;
	//	voltage_range = 5.0 - min_voltage;
	//	min_distance = 3.0;
	//	distance_range = 60.0 - min_distance;
    }
    //constructor
    public Sensors(int _channel, boolean _use_units) {
        channel = new AnalogChannel(_channel);
        //only use unit-specific variables if we're using units
        if (_use_units) {
            use_units = true;
//            min_voltage = _min_voltage;
//            voltage_range = _max_voltage - _min_voltage;
//            min_distance = _min_distance;
//            distance_range = _max_distance - _min_distance;
        }
    }
    // Just get the voltage.
    double GetVoltage() {
      return channel.getVoltage();
    }
    /* GetRangeInInches
     * Returns the range in inches
     * Returns -1.0 if units are not being used
     * Returns -2.0 if the voltage is below the minimum voltage
     */

    double GetRangeInInches() {
        double range;
        //if we're not using units, return -1, a range that will most likely never be returned
        if (!use_units) {
            return -1.0;
        }
        range = channel.getVoltage();
//        if (range < min_voltage) {
//            return -2.0;
//        }
//        //first, normalize the voltage
//        range = (range - min_voltage) / voltage_range;
//        //next, denormalize to the unit range
//        range = (range * distance_range) + min_distance;
        range = range / 0.005;
        return range;
    }
}

