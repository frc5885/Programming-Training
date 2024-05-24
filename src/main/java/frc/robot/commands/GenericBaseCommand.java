// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Base.SubsystemAction;
import frc.robot.subsystems.Base.WCSubsystem;

public class GenericBaseCommand<BaseSubsystem extends WCSubsystem> extends Command {
  BaseSubsystem m_baseSubsystem;
  SubsystemAction m_goalState;

  /**
   * A generic base command that manipulates a given base subsystem.
   * <p>
   * The given subsystem must be a child of the {@link WCSubsystem}.
   * 
   * @param baseSubsystem The subsystem to be manipulated
   * @param goalState     The desired state/action for the given subsystem
   */
  public GenericBaseCommand(BaseSubsystem baseSubsystem, SubsystemAction goalState) {
    m_baseSubsystem = baseSubsystem;
    m_goalState = goalState;

    addRequirements(m_baseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    HashMap<SubsystemAction, Supplier<Void>> map = m_baseSubsystem.getActionsMapping();
    map.forEach((state, action) -> {
      if (m_goalState == state) {
        action.get();
      }
    });
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_baseSubsystem.setSubsystemAction(null);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
