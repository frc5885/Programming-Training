// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AimbotCmd extends Command {
  private SwerveSubsystem m_swerveSubsystem;
  private Pose2d m_speaker;
  private PIDController m_pidController;
  private Supplier<Translation2d> m_translation2d;

  /**
   * Flexible command for both autonomous and periodic periods.
   * {@code Allows aimbot while driving}.
   * 
   * @param swerveSubsystem The swerveSubsystem
   * @param translation2d   The x and y values of the left stick on the joystick
   */
  public AimbotCmd(SwerveSubsystem swerveSubsystem, Supplier<Translation2d> translation2d) {
    m_swerveSubsystem = swerveSubsystem;
    m_speaker = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(4).get().toPose2d()
        : AprilTagFields.k2024Crescendo.loadAprilTagLayoutField().getTagPose(7).get().toPose2d();
    m_pidController = new PIDController(1.4, 0.0, 0.105);
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);
    m_pidController.setTolerance(Math.PI / 90);
    m_translation2d = translation2d;

    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_swerveSubsystem.getPose();
    double x = m_speaker.getX() - currentPose.getX();
    double y = m_speaker.getY() - currentPose.getY();
    double theta = Math.atan2(y, x);
    double angularVelocity = m_pidController.calculate(currentPose.getRotation().getRadians(), theta)
        * m_swerveSubsystem.getMaximumAngularVelocity();
    m_swerveSubsystem.drive(m_translation2d.get().times(m_swerveSubsystem.getMaximumVelocity()), angularVelocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pidController.atSetpoint();
  }
}
