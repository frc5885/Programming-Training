// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of m_swerveSubsystem project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PathPlanner extends SubsystemBase {
  SwerveSubsystem m_swerveSubsystem;

  /** Creates a new PathPlanner. */
  public PathPlanner(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;

    setUpPathPlanner();
  }

  public void setUpPathPlanner() {
    AutoBuilder.configureHolonomic(m_swerveSubsystem::getPose, m_swerveSubsystem::resetOdometry,
        m_swerveSubsystem::getRobotVelocity, m_swerveSubsystem::setChassisSpeeds,
        new HolonomicPathFollowerConfig(m_swerveSubsystem.getMaximumVelocity(),
            m_swerveSubsystem.getMaximumAngularVelocity(),
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
        },
        m_swerveSubsystem);
  }

  public Command buildFollowPath(Pose2d pose2d) {
    return AutoBuilder.pathfindToPose(pose2d,
        new PathConstraints(m_swerveSubsystem.getMaximumVelocity(), m_swerveSubsystem.getMaximumVelocity(),
            m_swerveSubsystem.getMaximumVelocity(), m_swerveSubsystem.getMaximumVelocity()),
        0, 0);
  }

  @Override
  public void periodic() {
  }
}
