package frc.robot;

import edu.wpi.first.wpilibj.Counter;

public class Lidar {
	/*
	 * Adjust the Calibration Offset to compensate for differences in each unit.
	 * We've found this is a reasonably constant value for readings in the 25 cm to
	 * 600 cm range. You can also use the offset to zero out the distance between
	 * the sensor and edge of the robot.
	 */
	private static final int CALIBRATION_OFFSET = -18;
	private static int WARNING_PRINT_LIMIT = 5; // How many times the "No LIDAR Readings" error will be printed

	private Counter counter;

	/**
	 * CONSTRUCTOR
	 */
	public Lidar(int channel) {
		counter = new Counter(channel);
		
		counter.setMaxPeriod(1.0);
		counter.setSemiPeriodMode(true); // Configure for measuring rising to falling pulses in the PWM signal (0V - 5V)
		counter.reset();
	}

	/*
	 * Measure the Microseconds and Convert to Centimeters
	 */
	public double getDistance() {

		double distance;

		//If we haven't seen the first rising to falling pulse, then we have no measurement. This happens when there is no LIDAR plugged in.
		if (counter.get() < 1) { // The counter.get() function returns how many intervals of 10 ms. of a PWM voltage pulse is counted
			if (WARNING_PRINT_LIMIT-- > 0) {
				System.out.println("LIDAR: Waiting for Readings");
			}
			return 0;
		}

		/*
		 * The counter.getPeriod() function returns time in seconds. The hardware
		 * resolution of the LIDAR is microseconds. The LIDAR sends 5V for 10
		 * microseconds per cm of distance. 1 sec. = 1,000,000 ms. 10 ms. = 1 cm.
		 */
		distance = (counter.getPeriod() * 1000000.0 / 10.0) + CALIBRATION_OFFSET;

		System.out.println(distance); // Test Purposes

		return distance;
	}
}