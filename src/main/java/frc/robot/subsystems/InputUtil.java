package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;

public class InputUtil {
  /**
   * Deadbands a joystick value
   * 
   * @param value the joystick value to deadband
   * @return the deadbanded joystick value
   */
  public static double deadband(double value) {
    if (Math.abs(value) <= OperatorConstants.DEADBAND) {
      return 0d; // * 0d just returns 0 as a double *
    } 
    else {
      return value;
    }
  }
}