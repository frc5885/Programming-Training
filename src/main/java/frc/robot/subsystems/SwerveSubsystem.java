// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import org.littletonrobotics.junction.AutoLogOutput;


public class SwerveSubsystem extends SubsystemBase
{

  private final SwerveDrive m_swerveDrive;
  public final double m_maximumSpeed = Constants.SwerveConstants.kMaxChassisSpeedMetersPerSecond;
  public final double m_maxAngularVelocity = Constants.SwerveConstants.kMaxAngularSpeedRadiansPerSecond;

  private boolean m_fieldOriented = true;
  
  public SwerveSubsystem(File directory)
  {
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try
    {
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(m_maximumSpeed);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }

    m_swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    m_swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    m_swerveDrive.swerveController.setMaximumAngularVelocity(m_maxAngularVelocity);

  }

  public void drive(Translation2d translationVelocity, double angularVelocity)
  { 
    // ADD CODE HERE
    m_swerveDrive.drive(translationVelocity, angularVelocity, m_fieldOriented, false); //remove
    // Open loop is disabled since it shouldn't be used most of the time. //remove
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
  }

  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose()
  {
    return m_swerveDrive.getPose();
  }

  public void zeroGyro()
  {
    m_swerveDrive.zeroGyro();
  }

  public void setMotorBrake(boolean brake)
  {
    m_swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return m_swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return m_swerveDrive.getRobotVelocity();
  }

  public void lock()
  {
    m_swerveDrive.lockPose();
  }

  public Rotation2d getPitch()
  {
    return m_swerveDrive.getPitch();
  }

  public void setFieldOriented(boolean fieldOriented)
  {
    m_fieldOriented = fieldOriented;
  }

  public void toggleFieldOriented()
  {
    m_fieldOriented = !m_fieldOriented;
  }

  public boolean getFieldOriented()
  {
    return m_fieldOriented;
  }

  public double getMaximumVelocity()
  {
    return m_swerveDrive.getMaximumVelocity();
  }

  public double getMaximumAngularVelocity()
  {
    return m_swerveDrive.getMaximumAngularVelocity();
  }

}