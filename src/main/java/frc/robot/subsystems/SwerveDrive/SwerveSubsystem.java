// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderIDs;
import frc.robot.Constants.EncoderOffsets;
import frc.robot.Constants.SparkMaxIDs;
import frc.robot.subsystems.AngleUtil;
import frc.robot.subsystems.InputUtil;
import frc.robot.subsystems.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private final static SwerveSubsystem INSTANCE = new SwerveSubsystem();

  // Sparkmaxes/motors for each swerve module
  SparkMax frontLeftDrive = new SparkMax(SparkMaxIDs.FRONT_LEFT_DRIVE, MotorType.kBrushless);
  SparkMax frontRightDrive = new SparkMax(SparkMaxIDs.FRONT_RIGHT_DRIVE, MotorType.kBrushless);
  SparkMax backLeftDrive = new SparkMax(SparkMaxIDs.BACK_LEFT_DRIVE, MotorType.kBrushless);
  SparkMax backRightDrive = new SparkMax(SparkMaxIDs.BACK_RIGHT_DRIVE, MotorType.kBrushless);

  SparkMax frontLeftTurn = new SparkMax(SparkMaxIDs.FRONT_LEFT_TURN, MotorType.kBrushless);
  SparkMax frontRightTurn = new SparkMax(SparkMaxIDs.FRONT_RIGHT_TURN, MotorType.kBrushless);
  SparkMax backLeftTurn = new SparkMax(SparkMaxIDs.BACK_LEFT_TURN, MotorType.kBrushless);
  SparkMax backRightTurn = new SparkMax(SparkMaxIDs.BACK_RIGHT_TURN, MotorType.kBrushless);

  // Gyroscope
  PigeonIMU gyro = new PigeonIMU(18);

  // Encoders
  AnalogEncoder frontLeftEncoder = new AnalogEncoder(EncoderIDs.FRONT_LEFT_ENCODER);
  AnalogEncoder frontRightEncoder = new AnalogEncoder(EncoderIDs.FRONT_RIGHT_ENCODER);
  AnalogEncoder backLeftEncoder = new AnalogEncoder(EncoderIDs.BACK_LEFT_ENCODER);
  AnalogEncoder backRightEncoder = new AnalogEncoder(EncoderIDs.BACK_RIGHT_ENCODER);

  AnalogInput frontLeftAnalogInput = new AnalogInput(EncoderIDs.FRONT_LEFT_ENCODER);
  AnalogInput frontRightAnalogInput = new AnalogInput(EncoderIDs.FRONT_RIGHT_ENCODER);
  AnalogInput backLeftAnalogInput = new AnalogInput(EncoderIDs.BACK_LEFT_ENCODER);
  AnalogInput backRightAnalogInput = new AnalogInput(EncoderIDs.BACK_RIGHT_ENCODER);

  RelativeEncoder frontLeftDriveEncoder = frontLeftDrive.getEncoder();
  RelativeEncoder frontRightDriveEncoder = frontRightDrive.getEncoder();
  RelativeEncoder backLeftDriveEncoder = backLeftDrive.getEncoder();
  RelativeEncoder backRightDriveEncoder = backRightDrive.getEncoder();

  // Swerve Modules
  SwerveModule frontLeft = new SwerveModule(frontLeftDrive, frontLeftTurn, frontLeftEncoder,
      frontLeftAnalogInput, frontLeftDriveEncoder, EncoderOffsets.FRONT_LEFT_ENCODER_OFFSET);
  SwerveModule frontRight = new SwerveModule(frontRightDrive, frontRightTurn, frontRightEncoder,
      frontRightAnalogInput, frontRightDriveEncoder, EncoderOffsets.FRONT_RIGHT_ENCODER_OFFSET);
  SwerveModule backLeft = new SwerveModule(backLeftDrive, backLeftTurn, backLeftEncoder,
      backLeftAnalogInput, backLeftDriveEncoder, EncoderOffsets.BACK_LEFT_ENCODER_OFFSET);
  SwerveModule backRight = new SwerveModule(backRightDrive, backRightTurn, backRightEncoder,
      backRightAnalogInput, backRightDriveEncoder, EncoderOffsets.BACK_RIGHT_ENCODER_OFFSET);

  // Kinematics & Odometry
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;

  /**
   * Creates a new instance of this SwerveSubsytem.
   * This constructor is private since this class is a Singleton. External classes
   * should use the {@link #getInstance()} method to get the instance.
   */
  public SwerveSubsystem() {

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW; // TODO LOWER THIS AT COMP, SLOWS COMPUTATION

    kinematics = new SwerveDriveKinematics(
        new Translation2d(0.368, 0.368),
        new Translation2d(0.368, -0.368),
        new Translation2d(-0.368, 0.368),
        new Translation2d(-0.368, -0.368));

    gyro.setFusedHeading(0);

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Math.toRadians(gyro.getFusedHeading())),
        modulePositions());
  }

  public final SwerveModulePosition[] modulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public final SwerveModuleState[] moduleStates() {
    return new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  public void drive(double x, double y, double turn, boolean field, boolean turbo, boolean updatingOffsets) {
    double dturn = InputUtil.deadband(turn);
    double dx = InputUtil.deadband(x) / (turbo ? 1 : 2);
    double dy = InputUtil.deadband(y) / (turbo ? 1 : 2);

    if (field) {
      double gyroRads = Math.toRadians(-gyro.getFusedHeading());
      double temp = dy * Math.cos(gyroRads) + dx * Math.sin(gyroRads);
      dx = -dy * Math.sin(gyroRads) + dx * Math.cos(gyroRads);
      dy = temp;
    }

    if (!updatingOffsets) {
      SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(dx, dy, -dturn));

      states[0] = optimize(states[0], Rotation2d.fromDegrees(frontLeft.getAngle()));
      states[1] = optimize(states[1], Rotation2d.fromDegrees(frontRight.getAngle()));
      states[2] = optimize(states[2], Rotation2d.fromDegrees(backLeft.getAngle()));
      states[3] = optimize(states[3], Rotation2d.fromDegrees(backRight.getAngle()));

      frontLeft.drive(states[0].speedMetersPerSecond, states[0].angle.getDegrees());
      frontRight.drive(states[1].speedMetersPerSecond, states[1].angle.getDegrees());
      backLeft.drive(states[2].speedMetersPerSecond, states[2].angle.getDegrees());
      backRight.drive(states[3].speedMetersPerSecond, states[3].angle.getDegrees());

      SmartDashboard.putNumber("Front Left Desired Angle", AngleUtil.circleMod(states[0].angle.getDegrees()));
      SmartDashboard.putNumber("Front Right Desired Angle", AngleUtil.circleMod(states[1].angle.getDegrees()));
      SmartDashboard.putNumber("Backleft Desired Angle", AngleUtil.circleMod(states[2].angle.getDegrees()));
      SmartDashboard.putNumber("Backright Desired Angle", AngleUtil.circleMod(states[3].angle.getDegrees()));

      SmartDashboard.putNumber("Front Left Power", states[0].speedMetersPerSecond);
      SmartDashboard.putNumber("Front Right Power", states[1].speedMetersPerSecond);
      SmartDashboard.putNumber("Back Left Power", states[2].speedMetersPerSecond);
      SmartDashboard.putNumber("Back Right Power", states[3].speedMetersPerSecond);

      SmartDashboard.putNumber("Front Left Angle", frontLeft.getAngle());
      SmartDashboard.putNumber("Front Right Angle", frontRight.getAngle());
      SmartDashboard.putNumber("Back Left Angle", backLeft.getAngle());
      SmartDashboard.putNumber("Back Right Angle", backRight.getAngle());

      SmartDashboard.putNumber("Front Left Speed", frontLeft.getSpeed());
      SmartDashboard.putNumber("Front Right Speed", frontRight.getSpeed());
      SmartDashboard.putNumber("Back Left Speed", backLeft.getSpeed());
      SmartDashboard.putNumber("Back Right Speed", backRight.getSpeed());

      SmartDashboard.putBoolean("Front Left Optimized?", frontLeft.isOptimized());
      SmartDashboard.putBoolean("Front Right Optimized?", frontRight.isOptimized());
      SmartDashboard.putBoolean("Back Left Optimized?", backLeft.isOptimized());
      SmartDashboard.putBoolean("Back Right Optimized?", backRight.isOptimized());

      odometry.update(Rotation2d.fromDegrees(getAngle()), modulePositions());

      // odometry.getPoseMeters();
      SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getTranslation().getX());
      SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getTranslation().getY());
      SmartDashboard.putNumber("Pose Degrees", getAngle());
    } 
    else {
      stopMotors();
      System.out.println("Front Left: " + frontLeft.getAngle());
      System.out.println("Front Right: " + frontRight.getAngle());
      System.out.println("Back Left: " + backLeft.getAngle());
      System.out.println("Back Right: " + backRight.getAngle());
    }
  }

  public void stopMotors() {
    frontLeft.stopMotors();
    frontRight.stopMotors();
    backLeft.stopMotors();
    backRight.stopMotors();
  }

  public void zeroWheels() {
    frontLeft.steerToAngle(0);
    frontRight.steerToAngle(0);
    backLeft.steerToAngle(0);
    backRight.steerToAngle(0);
  }

  public void resetEncoders() {
    frontLeftDriveEncoder.setPosition(0);
    frontRightDriveEncoder.setPosition(0);
    backLeftDriveEncoder.setPosition(0);
    backRightDriveEncoder.setPosition(0);
  }

  public void resetGyro() {
    gyro.setFusedHeading(0);
  }

  public double getAngle() {
    return (gyro.getFusedHeading());
  }

  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public SwerveModule getFrontLeft() {
    return frontLeft;
  }

  public SwerveModule getFrontRight() {
    return frontRight;
  }

  public SwerveModule getBackLeft() {
    return backLeft;
  }

  public SwerveModule getBackRight() {
    return backRight;
  }

  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }

  /**
   * Returns the Singleton instance of this Swervesubsystem. This static method
   * should be used -- {@code SwerveSubsystem.getInstance();} -- by external
   * classes, rather than the constructor to get the instance of this class.
   */
  public static SwerveSubsystem getInstance() {
    return INSTANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}