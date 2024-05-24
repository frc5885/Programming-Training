// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Base.SubsystemAction;
import frc.robot.subsystems.Base.WCSubsystem;

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
  protected void initActionsMapping() {
    m_actionsMap.put(SubsystemAction.INTAKE, this::intake);
    m_actionsMap.put(SubsystemAction.OUTTAKE, this::outtake);
  }

  @Override
  protected double getBaseSpeed() {
    return 1.0;
  }

  public Void intake() {
    m_subsystemAction = SubsystemAction.INTAKE;
    return null;
  }

  public Void outtake() {
    m_subsystemAction = SubsystemAction.OUTTAKE;
    return null;
  }

  @Override
  protected void putDebugDataPeriodic() {
  }

  @Override
  public void periodic() {
    super.periodic();
    if (m_subsystemAction == SubsystemAction.INTAKE) {
      forwardMotors();
    } else if (m_subsystemAction == SubsystemAction.OUTTAKE) {
      reverseMotors();
    } else {
      stopMotors();
    }
  }
}
