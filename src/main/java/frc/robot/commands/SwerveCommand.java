// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends Command {

  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier translationXFunction;
  private final DoubleSupplier translationYFunction;
  private final DoubleSupplier angularRotationXFunction;

  /** Creates a new SwerveCommand. */
  public SwerveCommand(SwerveSubsystem swerve, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    m_swerveSubsystem = swerve;
    translationXFunction = translationX;
    translationYFunction = translationY;
    angularRotationXFunction = angularRotationX;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Make the robot move
    double x = translationXFunction.getAsDouble();
    double y = translationYFunction.getAsDouble();
    double angle = angularRotationXFunction.getAsDouble();

    Translation2d translationVelocity = new Translation2d(Math.pow(x, 2) * m_swerveSubsystem.getMaximumVelocity() * -Math.signum(x),
                                                          Math.pow(y, 2) * m_swerveSubsystem.getMaximumVelocity() * Math.signum(y));
    double angularVelocity = Math.pow(angle, 2) * m_swerveSubsystem.getMaximumAngularVelocity() * -Math.signum(angle);

    m_swerveSubsystem.drive(translationVelocity, angularVelocity, m_swerveSubsystem.getFieldOriented());
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
