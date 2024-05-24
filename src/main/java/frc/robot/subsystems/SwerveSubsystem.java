// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.PoseEstimator.PhotonVisionSystem;
import java.io.File;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrive m_swerveDrive;
  private boolean m_fieldOriented = true;
  private PhotonVisionSystem m_photonVisionSystem;

  public SwerveSubsystem(File directory, PhotonVisionSystem photonVisionSystem) {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
    // objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75, 1);
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(150.0 / 7.0, 1);

    try {
      m_swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(Constants.SwerveConstants.kMaxChassisSpeedMetersPerSecond, angleConversionFactor,
              driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    m_photonVisionSystem = photonVisionSystem;

    m_swerveDrive.setHeadingCorrection(false);
    m_swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    m_swerveDrive.swerveController
        .setMaximumAngularVelocity(Constants.SwerveConstants.kMaxAngularSpeedRadiansPerSecond);
  }

  public void drive(Translation2d translationVelocity, double angularVelocity) {
    if (getFieldOriented()) {
      m_swerveDrive.driveFieldOriented(
          new ChassisSpeeds(translationVelocity.getX(), translationVelocity.getY(), angularVelocity));
    } else {
      m_swerveDrive.drive(new ChassisSpeeds(translationVelocity.getX(), translationVelocity.getY(), angularVelocity));
    }
  }

  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public Rotation2d getPitch() {
    return m_swerveDrive.getPitch();
  }

  public void lock() {
    m_swerveDrive.lockPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public void zeroGyro() {
    m_swerveDrive.zeroGyro();
  }

  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds getFieldVelocity() {
    return m_swerveDrive.getFieldVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake) {
    m_swerveDrive.setMotorIdleMode(brake);
  }

  public void setFieldOriented(boolean fieldOriented) {
    m_fieldOriented = fieldOriented;
  }

  public void toggleFieldOriented() {
    m_fieldOriented = !m_fieldOriented;
  }

  public boolean getFieldOriented() {
    return m_fieldOriented;
  }

  public double getMaximumVelocity() {
    return m_swerveDrive.getMaximumVelocity();
  }

  public double getMaximumAngularVelocity() {
    return m_swerveDrive.getMaximumAngularVelocity();
  }

  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_swerveDrive.kinematics;
  }

  public SwerveModulePosition[] getModulePositions() {
    return m_swerveDrive.getModulePositions();
  }

  @Override
  public void periodic() {
    m_swerveDrive.swerveDrivePoseEstimator.update(getHeading(), getModulePositions());
    Pose2d estimatedPosition = m_swerveDrive.swerveDrivePoseEstimator.getEstimatedPosition();
    Optional<EstimatedRobotPose> estimatedGlobalPosition = m_photonVisionSystem
        .getEstimatedGlobalPoseShooter(estimatedPosition);
    if (estimatedGlobalPosition.isPresent()) {
      EstimatedRobotPose estimatedVisionPose = estimatedGlobalPosition.get();
      Pose2d visionRobotPose2d = estimatedVisionPose.estimatedPose.toPose2d();
      m_swerveDrive.swerveDrivePoseEstimator.addVisionMeasurement(visionRobotPose2d,
          estimatedVisionPose.timestampSeconds);
    }
  }
}