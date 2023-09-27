package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.ChassisMode;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{

    private DriveMode mode;

    private DriveHardware driveHardware;

    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;

    /**
     * @param chassisMode
     * @param driveMode
     */
    public DriveSubsystem(ChassisMode chassisMode, DriveMode driveMode) {
        if (chassisMode == ChassisMode.B_TEAM) {
            driveHardware = new DriveSparkMAX(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID);
        } else {
            driveHardware = new DriveVictor(DriveConstants.frontLeftID, DriveConstants.frontRightID, DriveConstants.backLeftID, DriveConstants.backRightID);
        }
        mode = driveMode;
    }

    public DriveSubsystem(ChassisMode chassisMode) {
        this(chassisMode, DriveMode.TANKDRIVE);
    }

    public DriveSubsystem() {
        this(ChassisMode.C_TEAM, DriveMode.TANKDRIVE);
    }
    

    public void setArcadeDriveControls(double forward, double rotation) {
        mode = DriveMode.ARCADEDRIVE;
		arcadeForward = forward;
        arcadeRotation = rotation;
	}

	public void setTankDriveControls(double left, double right) {
        mode = DriveMode.TANKDRIVE;
        leftTank = left;
        rightTank = right;
	}

    public void swapMode() {
        if(mode == DriveMode.TANKDRIVE) {
            mode = DriveMode.ARCADEDRIVE;
            setArcadeDriveControls(arcadeForward, arcadeRotation);
        } else {
            mode = DriveMode.TANKDRIVE;
            setTankDriveControls(leftTank, rightTank);
        }
    }

    public DriveMode getMode(){
        return mode;
    }

    @Override
    public void periodic(){
        if (mode == DriveMode.TANKDRIVE){
            driveHardware.tankDrive(leftTank, rightTank);
        }
        else {
            driveHardware.arcadeDrive(arcadeForward, arcadeRotation);
        }
    } 

}
