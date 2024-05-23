// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.*;
import frc.robot.io.*;
import frc.robot.subsystems.*;
import swervelib.math.SwerveMath;

import java.io.File;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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
  DriverController m_driverController;
  SwerveSubsystem m_swerveSubsystem;
  PathPlanner m_pathPlanner;
  IntakeSubsystem m_intakeSubsystem;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driverController = new DriverController(0);
    m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    m_swerveSubsystem
        .setDefaultCommand(
            Robot.isReal() ? new SwerveJoystickCommand(
                m_swerveSubsystem,
                () -> -m_driverController.getLeftYCorrected(),
                () -> m_driverController.getLeftXCorrected(),
                () -> -m_driverController.getRightXCorrected())
                : new SwerveJoystickCommand(
                    m_swerveSubsystem,
                    () -> m_driverController.getRawAxis(1),
                    () -> m_driverController.getRawAxis(0),
                    () -> m_driverController.getRawAxis(2)));
    m_pathPlanner = new PathPlanner(m_swerveSubsystem);
    m_intakeSubsystem = new IntakeSubsystem();

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    if (Robot.isReal()) {
      m_driverController.scheduleOnLeftTriggerTrue(new AimbotCmd(m_swerveSubsystem, m_driverController::getLeftTranslation2d));
      m_driverController.getBButton().whileTrue(m_pathPlanner.buildFollowPath(m_swerveSubsystem.getPose()));
    } else {
      m_driverController.button(1)
          .whileTrue(new AimbotCmd(m_swerveSubsystem, m_driverController::getLeftTranslation2d));
      m_driverController.button(2).whileTrue(m_pathPlanner.buildFollowPath(new Pose2d(5, 5, new Rotation2d())));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
