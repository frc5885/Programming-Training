// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Base;

import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class WCSubsystem extends SubsystemBase {
  protected SubsystemAction m_subsystemAction;
  private final List<MotorController> m_motors;
  private final double baseVoltage = 12.0;
  protected double positionSim = 0;
  protected HashMap<SubsystemAction, Supplier<Void>> m_actionsMap;

  /** Creates a new WCSubsystem. */
  protected WCSubsystem() {
    m_motors = initMotors();
    m_actionsMap = new HashMap<>();
    initActionsMapping();
  }

  protected abstract List<MotorController> initMotors();

  protected abstract void initActionsMapping();

  protected abstract double getBaseSpeed();

  protected final void forwardMotors() {
    for (MotorController motor : m_motors) {
      motor.setVoltage(getBaseSpeed() * baseVoltage);
    }
  }

  protected final void reverseMotors() {
    for (MotorController motor : m_motors) {
      motor.setVoltage(-getBaseSpeed() * baseVoltage);
    }
  }

  protected final void stopMotors() {
    for (MotorController motor : m_motors) {
      motor.setVoltage(0);
    }
  }

  public void stop() {
    m_subsystemAction = null;
  }

  public HashMap<SubsystemAction, Supplier<Void>> getActionsMapping() {
    return m_actionsMap;
  }

  public void setSubsystemAction(SubsystemAction subsystemAction) {
    m_subsystemAction = subsystemAction;
  }

  public boolean checkSubsystemAction(SubsystemAction subsystemAction) {
    return m_subsystemAction == subsystemAction;
  }

  protected abstract void putDebugDataPeriodic();

  @Override
  public void periodic() {
    if (Constants.isLogging) {
      putDebugDataPeriodic();
    }
  }
}
