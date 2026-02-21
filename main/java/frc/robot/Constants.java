// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
      // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class AutonConstants
  {

   public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; 
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    public static final boolean TWO_CONTROLLER_MODE = false;
  }
  public static class VisionConstants
  {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double AIM_KP = 0.08;
    public static final double AIM_KI = 0.001;
    public static final double AIM_KD = 0.002;
    public static final double AIM_TOLERANCE_DEGREES = 0.5;
    public static final double AIM_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = 4.0;
    // Linear calibration for Limelight distance inches: calibrated = raw * scale + offset.
    // Defaults fit two tape-measure points: (raw 68.0 -> true 68.0), (raw 103.5 -> true 105.0).
    public static final double LIMELIGHT_DISTANCE_SCALE = 1.0422535;
    public static final double LIMELIGHT_DISTANCE_OFFSET_INCHES = 0.0;
  }
  public static class CustomConstants
  {


  }

  public static final class ShooterConstants
  {
    public static final int SHOOTER_INTAKE_MOTOR_ID = 20;
    public static final int MIDDLE_SHOOTER_MOTOR_ID = 21;

    public static final boolean SHOOTER_INTAKE_INVERTED = true;
    public static final boolean MIDDLE_SHOOTER_INVERTED = true;

    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final double SHOOTER_INTAKE_POWER = 0.99;
    public static final double SHOOTER_POWER_AT_5FT = 1.00;
    public static final double SHOOTER_POWER_AT_10FT = 0.50;
    public static final double SHOOTER_POWER_AT_15FT = 1.00;
    public static final double SHOOTER_POWER_AT_20FT = 1.00;
    public static final double SHOOTER_POWER_TOLERANCE = 0.02;
    public static final double SHOOTER_INTAKE_START_DELAY_SECONDS = 1;

    public static final double SHOOTER_KP = 0.00025;
    public static final double SHOOTER_KI = 0.0;
    public static final double SHOOTER_KD = 0.0;
    public static final double SHOOTER_KF = 0.0;

    public static final double SHOOTER_INTAKE_KP = 1.0;
    public static final double SHOOTER_INTAKE_KI = 0.0;
    public static final double SHOOTER_INTAKE_KD = 0.0;
    public static final double SHOOTER_INTAKE_KF = 0.000;
  }

  public static final class IntakeConstants
  {
    public static final int PIVOT_MOTOR_ID = 30;
    public static final int WHEEL_MOTOR_ID = 31;
    public static final boolean PIVOT_INVERTED = false;
    public static final boolean WHEEL_INVERTED = false;
    public static final int CURRENT_LIMIT_AMPS = 40;
    public static final double PIVOT_POWER = 0.30;
    public static final double PIVOT_IN_ANGLE_DEGREES = 20.0;
    public static final double PIVOT_OUT_ANGLE_DEGREES = 154.0;
    public static final double PIVOT_ANGLE_TOLERANCE_DEGREES = 2.0;
    public static final double WHEEL_POWER = 0.80;
  }
}
