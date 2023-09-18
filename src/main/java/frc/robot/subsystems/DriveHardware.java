package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveHardware {

    protected MotorControllerGroup rightMotorControllerGroup;
    protected MotorControllerGroup leftMotorControllerGroup;
    protected DifferentialDrive drive;
	// protected final PigeonIMU pigeon = new PigeonIMU(DriveConstants.pigeonID);
    protected final double angleOffset = 0;

    public DriveHardware() {}
    
    // public double GetDriveAngle() {
	// 	return pigeon.getYaw()-angleOffset;
	// }

    public void tankDrive(double leftTank, double rightTank) {
        drive.tankDrive(leftTank, rightTank);
    }

    public void arcadeDrive(double forward, double rotation) {
        drive.arcadeDrive(forward, -rotation);
    }
}
