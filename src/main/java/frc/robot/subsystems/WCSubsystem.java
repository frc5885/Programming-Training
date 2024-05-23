// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class WCSubsystem extends SubsystemBase {
  private final List<MotorController> motors;
  private final double baseVoltage = 12.0;
  protected SubsystemAction subsystemAction;
  protected double positionSim = 0;

  /** Creates a new WCSubsystem. */
  protected WCSubsystem() {
    motors = initMotors();
  }

  protected abstract List<MotorController> initMotors();

  protected abstract double getBaseSpeed();

  protected final void forwardMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(getBaseSpeed() * baseVoltage);
    }
  }

  protected final void reverseMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(-getBaseSpeed() * baseVoltage);
    }
  }

  protected final void stopMotors() {
    for (MotorController motor : motors) {
      motor.setVoltage(0);
    }
  }

  public void stop() {
    subsystemAction = null;
  }

  protected abstract void putDebugDataPeriodic();

  @Override
  public void periodic() {
    if (Constants.isLogging) {
      putDebugDataPeriodic();
    }
  }
}
