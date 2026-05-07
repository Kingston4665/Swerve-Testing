package frc.robot.subsystems;

public class InputUtil {
    /**
     * Deadbands a joystick value
     * @param value the joystick value to deadband
     * @return the deadbanded joystick value
     */
    public static double deadband(double value) {
        if (value >= -0.1 && value <= 0.1) return 0d;
        return value;
    }
}