package frc.robot.io;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.function.BooleanSupplier;

public class WCDualsenseController extends CommandPS5Controller {

  protected final JoystickButton crossButton =
      new JoystickButton(getHID(), PS5Controller.Button.kCross.value);
  protected final JoystickButton circleButton =
      new JoystickButton(getHID(), PS5Controller.Button.kCircle.value);
  protected final JoystickButton triangleButton =
      new JoystickButton(getHID(), PS5Controller.Button.kTriangle.value);
  protected final JoystickButton squareButton =
      new JoystickButton(getHID(), PS5Controller.Button.kSquare.value);
  protected final JoystickButton leftBumper =
      new JoystickButton(getHID(), PS5Controller.Button.kL1.value);
  protected final JoystickButton rightBumper =
      new JoystickButton(getHID(), PS5Controller.Button.kR1.value);
  protected final JoystickButton leftStick =
      new JoystickButton(getHID(), PS5Controller.Button.kL3.value);
  protected final JoystickButton rightStick =
      new JoystickButton(getHID(), PS5Controller.Button.kR3.value);
  protected final JoystickButton backButton =
      new JoystickButton(getHID(), PS5Controller.Button.kCreate.value);
  protected final JoystickButton startButton =
      new JoystickButton(getHID(), PS5Controller.Button.kOptions.value);

  public WCDualsenseController(int port) {
    super(port);
  }

  public JoystickButton getCrossButton() {
    return crossButton;
  }

  public JoystickButton getCircleButton() {
    return circleButton;
  }

  public JoystickButton getTriangleButton() {
    return triangleButton;
  }

  public JoystickButton getSquareButton() {
    return squareButton;
  }

  public JoystickButton getLeftBumper() {
    return leftBumper;
  }

  public JoystickButton getRightBumper() {
    return rightBumper;
  }

  public JoystickButton getLeftStick() {
    return leftStick;
  }

  public JoystickButton getRightStick() {
    return rightStick;
  }

  public JoystickButton getBackButton() {
    return backButton;
  }

  public JoystickButton getStartButton() {
    return startButton;
  }

  @Override
  public double getLeftY() {
    return -super.getLeftY();
  }

  @Override
  public double getRightY() {
    return -super.getRightY();
  }

  public void scheduleOnRightTrigger(Command command) {
    scheduleOnRightTrigger(command, 0.1);
  }

  public void scheduleOnRightTrigger(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getR2Axis() > minimumInput);
  }

  public void scheduleOnLeftTriggerTrue(Command command) {
    scheduleOnLeftTriggerTrue(command, 0.1);
  }

  public void scheduleOnLeftTriggerFalse(Command command) {
    scheduleOnLeftTriggerFalse(command, 0.1);
  }

  public void scheduleOnLeftTriggerTrue(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getL2Axis() > minimumInput);
  }

  public void scheduleOnLeftTriggerFalse(Command command, double minimumInput) {
    scheduleOnInput(command, () -> getL2Axis() <= minimumInput);
  }

  private void scheduleOnInput(Command command, BooleanSupplier condition) {
    EventLoop looper = CommandScheduler.getInstance().getDefaultButtonLoop();
    looper.bind(
        new Runnable() {
          private boolean wasTriggered = condition.getAsBoolean();

          @Override
          public void run() {
            boolean isTriggered = condition.getAsBoolean();
            if (!wasTriggered && isTriggered) {
              command.schedule();
            } else if (wasTriggered && !isTriggered) {
              command.cancel();
            }
            wasTriggered = isTriggered;
          }
        });
  }
}
