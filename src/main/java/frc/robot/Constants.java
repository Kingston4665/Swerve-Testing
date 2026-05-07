// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OperatorConstants {
    public static final int DRIVER = 0;
    public static final double DEADBAND = 0.1;
  }
  
  public static final class SparkMaxIDs {
    public static final int FRONT_LEFT_DRIVE = 1;
    public static final int FRONT_RIGHT_DRIVE = 2;
    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_RIGHT_DRIVE = 4;
    public static final int FRONT_LEFT_TURN= 5;
    public static final int FRONT_RIGHT_TURN = 6;
    public static final int BACK_LEFT_TURN = 7;
    public static final int BACK_RIGHT_TURN = 8;
  }

  public static final class EncoderIDs {
    public static final int FRONT_LEFT_ENCODER = 0;
    public static final int FRONT_RIGHT_ENCODER = 1;
    public static final int BACK_LEFT_ENCODER = 2;
    public static final int BACK_RIGHT_ENCODER = 3;
  }

  public static final class EncoderOffsets {
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0;
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0;
    public static final double BACK_LEFT_ENCODER_OFFSET = 0;
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0;
  }

  public static final class SwerveConstants {
    public static final double GEAR_RATIO = 0;
    public static final double WHEEL_DIAMETER = 0; // In meters
  }
}