package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.math.MathUtil.clamp;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

  private SparkMax drive;
  private SparkMax turn;

  private AnalogEncoder encoder;
  private AnalogInput encoderAnalogInput;
  private RelativeEncoder driveEncoder;

  private boolean optimized;

  private double encoderOffset;

  /**
   * Constructs a new swerve module
   * 
   * @param drive        motor controller for drive motor
   * @param turn         motor controller for steer motor
   * @param steerEncoder steer encoder - not from spark max
   * @param angleOffset  module angle offset
   */
  public SwerveModule(SparkMax drive, SparkMax turn, AnalogEncoder encoder, AnalogInput encoderAnalogInput,
      RelativeEncoder driveEncoder, double encoderOffset) {
    this.drive = drive;
    this.turn = turn;
    this.encoder = encoder;
    this.encoderAnalogInput = encoderAnalogInput;
    this.encoderOffset = encoderOffset;
    this.driveEncoder = driveEncoder;
  }

  /**
   * Drive the swerve module with power and angle
   * 
   * @param power Motor power of the swerve module
   * @param angle Desired angle
   */
  public void drive(double power, double angle) {
    SpeedAnglePair powerAndAngle = optimize(power, angle);
    drive.set(powerAndAngle.getX());
    steerToAngle(powerAndAngle.getY());
  }

  public void stopMotors() {
    drive.stopMotor();
    turn.stopMotor();
  }

  /**
   * Steer swerve module to angle between [0,360)
   * 
   * @param angle the desired angle of the swerve module
   */
  public void steerToAngle(double angle) {
    steerToAngle(angle, 0.008);
  }

  /**
   * Steer swerve module to angle between [0,360)
   * 
   * @param angle the desired angle of the swerve module
   * @param p     the p value to use
   */
  public void steerToAngle(double angle, double p) {

    double error = angle - getAngle();
    if (error < -180) {
      error += 360;
    }
    if (error > 180) {
      error -= 360;
    }

    error *= p;
    error = clamp(error, -1, 1);
    boolean errorNegative = error < 0;

    if (Math.abs(error) >= 0.01)
      error = Math.max(Math.abs(error / 3), 0.015) * (errorNegative ? -1 : 1);

    if (Math.abs(error) > 0) {
      turn.set(error);
    } 
    else {
      turn.stopMotor();
    }
  }

  /**
   * Optimizes swerve module angles
   * 
   * @param power swerve power
   * @param angle swerve angle
   * @return A Vector2 containing the optimized angle and power
   */
  private SpeedAnglePair optimize(double power, double angle) {

    // TODO: Fix optimization
    // double delta = AngleUtil.circleMod(angle)-getAngle();
    //
    // if (Math.abs((delta)) > 90.0) {
    // optimized = true;
    // return new SpeedAnglePair(-power, AngleUtil.circleMod((angle + 180) % 360));
    // }
    //
    // optimized = false;
    return new SpeedAnglePair(power, AngleUtil.circleMod(angle));
  }

  // thanks 2910, very cool

  /**
   * Gets swerve module angle
   * 
   * @return the swerve module angle
   */
  public double getAngle() {
    double angle = (1.0 - encoderAnalogInput.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI;

    angle = Math.toDegrees(angle);
    angle = AngleUtil.circleMod(angle - encoderOffset);

    return angle;
  }

  /**
   * Gets the state for odometry
   * 
   * @return the swerve module state
   */
  public SwerveModuleState getState() {
    // double angle = optimized ? AngleUtil.circleMod(getAngle() + 180) - 180 :
    // getAngle();
    return new SwerveModuleState(
        (driveEncoder.getVelocity() / SwerveConstants.GEAR_RATIO) * SwerveConstants.WHEEL_DIAMETER * Math.PI / 60,
        Rotation2d.fromDegrees(getAngle()));
  }

  public SparkMax getDrive() {
    return drive;
  }

  public SparkMax getTurn() {
    return turn;
  }

  public AnalogEncoder getEncoder() {
    return encoder;
  }

  public double getEncoderOffset() {
    return encoderOffset;
  }

  public void setEncoderOffset(double encoderOffset) {
    this.encoderOffset = encoderOffset;
  }

  public double getSpeed() {
    return drive.get();
  }

  public boolean isOptimized() {
    return optimized;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      Rotation2d.fromDegrees(getAngle()));
  }
}
