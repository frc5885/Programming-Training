package frc.robot.io;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

public class DriverController extends WCDualsenseController {

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
    
}
