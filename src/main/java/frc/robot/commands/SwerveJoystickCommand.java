// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCommand extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier m_translationXFunction;
  private final DoubleSupplier m_translationYFunction;
  private final DoubleSupplier m_angularRotationXFunction;

  /** Creates a new SwerveJoystickCommand. */
  public SwerveJoystickCommand(SwerveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    m_swerveSubsystem = swerve;
    m_translationXFunction = translationX;
    m_translationYFunction = translationY;
    m_angularRotationXFunction = angularRotationX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ADD CODE HERE
    double x = m_translationXFunction.getAsDouble(); //remove
    double y = m_translationYFunction.getAsDouble(); //remove
    double angle = m_angularRotationXFunction.getAsDouble(); //remove

    double xVelocity = x * m_swerveSubsystem.getMaximumVelocity(); //remove
    double yVelocity = y * m_swerveSubsystem.getMaximumVelocity(); //remove
    double angularVelocity = angle * m_swerveSubsystem.getMaximumAngularVelocity(); //remove

    Translation2d translationVelocity = new Translation2d(xVelocity, yVelocity); //remove

    m_swerveSubsystem.drive(translationVelocity, angularVelocity); //remove
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
