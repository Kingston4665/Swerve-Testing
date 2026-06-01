// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem driveBase = new SwerveSubsystem();

  // Creates the Xbox Controllers
  private final CommandXboxController driverController = new CommandXboxController(Constants.OperatorConstants.DRIVER);
  private final CommandXboxController copilotController = new CommandXboxController(Constants.OperatorConstants.COPILOT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);

    // Builds an auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    driveBase.setDefaultCommand(
      new RunCommand(
        () -> driveBase.drive(
            -driverController.getLeftY(),
            -driverController.getLeftX(),
            driverController.getRightX(),
						true, // Field Oriented is always true for driver control
            driverController.rightTrigger().getAsBoolean() // * Turbo button *
					),
        driveBase
			)
		);
  }	

  private final SendableChooser<Command> autoChooser;

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    driveBase.resetGyro();
    driveBase.resetEncoders();
    driveBase.getOdometry().resetPosition(new Rotation2d(), driveBase.modulePositions(), new Pose2d());

    return autoChooser.getSelected();
  }

	// Resets the heading of the gyroscope
  public void resetGyro() {
    driveBase.resetGyro();
  }

	// Adds 180 degrees to the gyroscope if we are on red alliance to make the 
	// gyroscope and controls correct so the driver doesn't have to do it manually 
	public void allianceRelativeGyroscopeControl() {
		driveBase.allianceRelativeGyroscopeControl();
	}

	// Match Start Protocol
	public void matchStartProtocol(){
		driveBase.zeroWheels();
    driveBase.resetEncoders();
    driveBase.getOdometry().resetPosition(new Rotation2d(), driveBase.modulePositions(), new Pose2d());
	}
}
