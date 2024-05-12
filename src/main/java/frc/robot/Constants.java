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
  public static class DriverControllerConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadbandLeftX = 0.05;
    public static final double kDeadbandLeftY = 0.05;
    public static final double kDeadbandRightX = 0.05;
    public static final double kDeadbandRightY = 0.05;
  }
  
  public static class SwerveConstants {
    public static final double kMaxChassisSpeedMetersPerSecond = 4.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
    public static final double kMaxAngularAccelerationRadPerSecondSquared = 2 * Math.PI;
    
  }

  // for advantagekit
  public static final Mode currentMode = Mode.REAL;
  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

}
