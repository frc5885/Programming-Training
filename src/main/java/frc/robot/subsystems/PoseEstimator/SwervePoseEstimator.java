// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.PoseEstimator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

// This object is created in the WCRobot class
public class SwervePoseEstimator extends SubsystemBase {
  private final Supplier<Rotation2d> m_rotationSupplier;
  private final Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private final SwerveDrivePoseEstimator m_poseEstimator;
  private final SwerveSubsystem m_swerveDrive;
  private final PhotonVisionSystem m_photonVision;
  long lastRan = 0;
  int delay = 100;

  /** Creates a new TankDrivePoseEstimator. */
  public SwervePoseEstimator(SwerveSubsystem swerveDrive, PhotonVisionSystem photonVision) {

    m_rotationSupplier = swerveDrive::getHeading;
    m_swerveModulePositionSupplier = swerveDrive::getModulePositions;

    m_poseEstimator = new SwerveDrivePoseEstimator(
        swerveDrive.getSwerveDriveKinematics(),
        m_rotationSupplier.get(),
        m_swerveModulePositionSupplier.get(),
        new Pose2d(),
        AutoConstants.PoseEstimatorConstants.kEncoderMeasurementStdDevs,
        AutoConstants.PoseEstimatorConstants.kVisionMeasurementStdDevs);

    m_swerveDrive = swerveDrive;

    m_photonVision = photonVision;

    reset();
  }

  @Override
  public void periodic() {
    // long now = System.currentTimeMillis();
    // if (now - lastRan > delay) {
    // lastRan = now;
    // This method will be called once per scheduler run

    // Update the WPI pose estimator with the latest rotation and position
    // measurements from swerve
    // system
    m_poseEstimator.update(m_rotationSupplier.get(), m_swerveModulePositionSupplier.get());
    Pose2d estimatedPosition = getPose();
    // if (WCLogger.isEnabled) {
    Logger.recordOutput(this.getClass().getSimpleName() + "/EstimatedPose", estimatedPosition);
    // }

    // Update the WPI pose estimator with the latest vision measurements from photon
    // vision if they
    // are present
    Optional<EstimatedRobotPose> estimatedGlobalPosition = m_photonVision
        .getEstimatedGlobalPoseShooter(estimatedPosition);
    // WCLogger.putBoolean(this, "IsEstimatedPositionPresent",
    // estimatedGlobalPosition.isPresent());
    if (estimatedGlobalPosition.isPresent()) {

      // have to call .get() to get the value from the optional
      EstimatedRobotPose estimatedVisionPose = estimatedGlobalPosition.get();

      Pose3d pose3d = estimatedVisionPose.estimatedPose;
      Pose2d pose2d = pose3d.toPose2d();

      // actually add the vision measurement
      m_poseEstimator.addVisionMeasurement(pose2d, estimatedVisionPose.timestampSeconds);

      // if (WCLogger.isEnabled) {
      // Logger.recordOutput(this.getClass().getSimpleName() +
      // "/VisionEstimatedPose3D", pose3d);
      // Logger.recordOutput(this.getClass().getSimpleName() + "/VisionEstimatedPose",
      // pose2d);
      // }
    }
    // }
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d newPose) {
    m_swerveDrive.zeroGyro();
    m_poseEstimator.resetPosition(
        m_rotationSupplier.get(), m_swerveModulePositionSupplier.get(), newPose);
  }

  public void reset() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    resetPose(
        new Pose2d(
            alliance == Alliance.Blue ? 0 : 17,
            0,
            alliance == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI)));
  }
}
