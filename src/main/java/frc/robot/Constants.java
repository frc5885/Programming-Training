// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
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
    public static final double kTrackWidthMeters = Units.inchesToMeters(24.0);
    public static final double kWheelBaseWidthMeters = Units.inchesToMeters(24.0);
    public static final double kMaxChassisSpeedMetersPerSecond = 4.2;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.0;
    public static final double kMaxAngularAccelerationRadPerSecondSquared = 2 * Math.PI;
  }

  public class CameraConstants {
    public class Intake {
      public static final String kCameraName = "Arducam_5MP_Camera_Module";
    }

    public class ShooterBlue {
      public static final String kCameraName = "USB_Camera";
      // Higher offset = further away
      // Blue
      public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 13.0);
      // Red
      // public static final double kCameraPositionX = Units.inchesToMeters(11.56 +
      // 0.39 + 12.0);
      public static final double kCameraPositonY = 0.0;
      public static final double kCameraPositionZ = Units.inchesToMeters(13.876 + 1.9);
      public static final double kCameraRoll = 0.0;
      public static final double kCameraPitch = Units.degreesToRadians(15);
      public static final double kCameraYaw = 0.0;
    }

    public class ShooterRed {
      public static final String kCameraName = "USB_Camera";
      // Higher offset = further away
      // Blue
      // public static final double kCameraPositionX = Units.inchesToMeters(11.56 +
      // 0.39 + 14.0);
      // Red
      public static final double kCameraPositionX = Units.inchesToMeters(11.56 + 0.39 + 13.0);
      public static final double kCameraPositonY = 0.0;
      public static final double kCameraPositionZ = Units.inchesToMeters(13.876 + 1.9);
      public static final double kCameraRoll = 0.0;
      public static final double kCameraPitch = Units.degreesToRadians(15);
      public static final double kCameraYaw = 0.0;
    }
  }

  public class AutoConstants {
    public static final class AutoStartingPositions {
      public static final Pose2d kLeftOffSubwoofer = new Pose2d(0.42, 7.0, new Rotation2d(0.0));
      public static final Pose2d kLeftOnSubwoofer = new Pose2d(0.639, 6.685, new Rotation2d(1.064));
    }

    /**
     * Constants related to the robots pose estimation. This includes camera
     * positions, kalman filter
     * constants, and other pose estimation related constants.
     */
    public static final class PoseEstimatorConstants {
      ///////////////////
      // Camera Positions
      //
      // All positions are measured from the robots center to camera sensor, in order
      // of IDs.
      // Rotation is in degrees, relative to robot center pointing forward. Try to
      // keep this as close to 45/90/135/180/etc. as possible.
      public static final Transform3d[] kCameraPositionMeters = {
          new Transform3d(
              new Translation3d(34 / 100, Units.inchesToMeters(0), Units.inchesToMeters(0)),
              new Rotation3d(0, 0, 0)),
          // new Transform3d(
          // new Translation3d(
          // Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
          // new Rotation3d(0, 0, 0)),
      };

      //////////////////////////
      // Kalman filter constants
      //
      // Higher number means less trust in the model (more spread in the measurement)
      // Order is {x, y, theta}
      // Encoder measurements are from the encoders inside the Rev NEO motors.
      // Vision measurements are from the NoodleVision systems.
      public static final Matrix<N3, N1> kEncoderMeasurementStdDevs = VecBuilder.fill(0.03, 0.03, 0.03);
      public static final Matrix<N3, N1> kVisionMeasurementStdDevs = VecBuilder.fill(0.15, 0.15, 0.15);
    }

    public static final double kAutoSpeedMultiplier = 0.25;

    // Pathplanner
    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5.0, 0.0, 0.0), // Translation constants
        new PIDConstants(1.3, 0.0, 0.0), // Rotation constants
        SwerveConstants.kMaxChassisSpeedMetersPerSecond,
        Math.hypot(
            SwerveConstants.kTrackWidthMeters / 2,
            SwerveConstants.kWheelBaseWidthMeters
                / 2), // Drive base radius (distance from center to furthest module)
        new ReplanningConfig());
  }

  public static class IntakeConstants {
    public static final int kIntakeLeft = 31;
    public static final int kIntakeRight = 30;
  }

  // for advantagekit
  public static Boolean isLogging = false;
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
