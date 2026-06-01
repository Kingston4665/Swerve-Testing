package frc.robot.subsystems;

public class AngleUtil {

  /**
   * Optimize the rotation angle.
   *
   * The return rotation angle is optimized by ensuring we rotate no more than 180
   * degrees.
   *
   * @param rotationInDegrees
   * @return the optimized rotation angle
   */
  public static double optimizeRotation(double rotationInDegrees) {

    // Ensure the rotation is in the interval (-360, 360)
    double ra = circleMod(rotationInDegrees);

    // If the angle is greater than 180 degrees in either direction,
    // rotate the other direction.
    if (ra > 180.0) {
      ra -= 360;
    } 
		else if (ra < -180.0) {
      ra += 360.0;
    }

    return ra;
  }

  /**
   * Optimize the drive angle and power.
   *
   * This function calculates the minimum angular distance between the current
   * angle and the desired angle. The minimum angle could be one of four possibilities:
   *
   * 1) rotating clockwise, with the robot remaining in the forward direction
   * 2) rotating counterclockwise, with the robot remaining in the forward direction
   * 3) rotating clockwise, wih the robot in the reverse direction
   * 4) rotating counterclockwise, with the robot in the reverse direction
   *
   * If the shortest path results in the robot in the reversed position, the power
   * is inverted.
   *
   * See https://www.chiefdelphi.com/t/efficient-swerve-drive/153593/2
   *
   * @param power
   * @param currentAngle
   * @param desiredAngle
   * @return - Vector, with power in the X coord and angle in the Y coord
   */
  public static SpeedAnglePair optimizeDriveAngleAndPower(double power, double currentAngle, double desiredAngle) {

    currentAngle = circleMod(currentAngle);
    desiredAngle = circleMod(desiredAngle);
    double reverseDesiredAngle = circleMod(desiredAngle + 180.0);

    // Calculate the angular distance if the robot remains forward facing
    double distanceForward = distance(currentAngle, desiredAngle);

    // Calculate the angular distance if the robot is reversed
    double distanceReverse = distance(currentAngle, reverseDesiredAngle);

    SpeedAnglePair retVal;
    if (distanceReverse < distanceForward) {
      retVal = new SpeedAnglePair(-power, reverseDesiredAngle);
    } 
    else {
      retVal = new SpeedAnglePair(power, desiredAngle);
    }

    return retVal;
  }

  /*
   * // this is the current optimization method, put here just for testing
   * public static SpeedAnglePair OLD_optimize(double power, double currentAngle,
   * double desiredAngle) {
   * desiredAngle = (desiredAngle + 360) % 360; // make sure it is between 0-360;
   * 
   * double error = desiredAngle - currentAngle;
   * if (Math.abs(error) > 90 && Math.abs(error) < 270) {
   * desiredAngle = (desiredAngle + 180) % 360;
   * power = -power;
   * }
   * 
   * return new SpeedAnglePair(power, desiredAngle);
   * }
   */

  /**
   * Calculate the distance in degrees between two angles, a and b.
   *
   * @param a - an angle in degrees
   * @param b - an angle in degrees
   *
   * @return - angular distance between the two angles, as an absolute value.
   */
  public static double distance(double a, double b) {
    return Math.min(circleMod(a - b), circleMod(b - a));
  }

  /**
   * Return the value modulo 360, with a guarantee to be in the interval [0, 360).
   * Note: the '%' (remainder) operation in Java can return a negative value.
   *
   * @param angleInDegrees
   */
  public static double circleMod(double angleInDegrees) {
    while (angleInDegrees < 0.0) {
      angleInDegrees += 360.0;
    }
    return angleInDegrees % 360.0;
  }
}
