// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.io.*;
import frc.robot.subsystems.*;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // ADD CODE HERE (define subsystem and controller)
  private final SwerveSubsystem m_swerveDrive;  //remove
  private final DriverController m_driverController;  //remove

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize subsystems here
    // ADD CODE HERE (initialize subsystem and controller, create command)
    m_swerveDrive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));  //remove
    m_driverController = new DriverController(DriverControllerConstants.kDriverControllerPort); //remove

    SwerveJoystickCommand swerveCommand = new SwerveJoystickCommand(m_swerveDrive,  //remove
                                                    () -> m_driverController.getLeftXCorrected(),  //remove
                                                    () -> m_driverController.getLeftYCorrected(),  //remove
                                                    () -> m_driverController.getRightXCorrected()); //remove

    m_swerveDrive.setDefaultCommand(swerveCommand);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Actual auto command would go here, or a function to get the selected command from a chooser.
    return Commands.none();
  }
}
