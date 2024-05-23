// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends WCSubsystem {
  CANSparkMax m_intakeMotorRight;
  CANSparkMax m_intakeMotorLeft;

  @Override
  protected List<MotorController> initMotors() {
    m_intakeMotorLeft = new CANSparkMax(IntakeConstants.kIntakeLeft, MotorType.kBrushless);
    m_intakeMotorRight = new CANSparkMax(IntakeConstants.kIntakeRight, MotorType.kBrushless);
    m_intakeMotorLeft.setInverted(true);
    return List.of(m_intakeMotorLeft, m_intakeMotorRight);
  }

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  public void intake() {
    subsystemAction = SubsystemAction.INTAKE;
  }

  public void outtake() {
    subsystemAction = SubsystemAction.OUTTAKE;
  }

  @Override
  protected void putDebugDataPeriodic() {
  }

  @Override
  public void periodic() {
    super.periodic();
    if (subsystemAction == SubsystemAction.INTAKE) {
      forwardMotors();
    } else if (subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else {
      stopMotors();
    }
  }
}
