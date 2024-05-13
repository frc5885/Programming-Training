// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    double x = m_translationXFunction.getAsDouble(); 
    double y = m_translationYFunction.getAsDouble(); 
    double angle = m_angularRotationXFunction.getAsDouble(); 

    double xVelocity = -y * m_swerveSubsystem.getMaximumVelocity(); 
    double yVelocity = x * m_swerveSubsystem.getMaximumVelocity(); 
    double angularVelocity = -angle * m_swerveSubsystem.getMaximumAngularVelocity(); 

    Translation2d translationVelocity = new Translation2d(xVelocity, yVelocity); 

    // the controls need to be mirrored for both alliances for robot oriented, so we can just mirror them if we're on blue or in robot oriented
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue || !m_swerveSubsystem.getFieldOriented()) {
      // rotate the translation velocity by 180 degrees to mirror it for the blue side
      translationVelocity = translationVelocity.rotateBy(Rotation2d.fromDegrees(180)); 
    }

    m_swerveSubsystem.drive(translationVelocity, angularVelocity); 
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

