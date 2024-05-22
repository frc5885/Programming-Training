package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class DriverController extends WCXboxController {

    public DriverController(int port) {
        super(port);
    }

    public double getLeftXCorrected() {
        return MathUtil.applyDeadband(super.getLeftX(), Constants.DriverControllerConstants.kDeadbandLeftX);
    }

    public double getLeftYCorrected() {
        return MathUtil.applyDeadband(super.getLeftY(), Constants.DriverControllerConstants.kDeadbandLeftY);
    }

    public double getRightXCorrected() {
        return MathUtil.applyDeadband(super.getRightX(), Constants.DriverControllerConstants.kDeadbandRightX);
    }

    public double getRightYCorrected() {
        return MathUtil.applyDeadband(super.getRightY(), Constants.DriverControllerConstants.kDeadbandRightY);
    }

    public Translation2d getLeftTranslation2d() {
        return new Translation2d(getLeftXCorrected(), getLeftYCorrected());
    }

}
