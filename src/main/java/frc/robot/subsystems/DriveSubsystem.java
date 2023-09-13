package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{

    public enum ChassisMode {
        B_TEAM,
        C_TEAM
    }

    public enum DriveMode {
        TANKDRIVE,
        ARCADEDRIVE
    }

    private DriveHardware driveHardware;

    private DriveMode mode;
    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;

    public DriveSubsystem(ChassisMode chassisMode, DriveMode driveMode) {
        if (chassisMode == ChassisMode.B_TEAM) {
            driveHardware = new DriveSparkMAX(DriveConstants.frontLeftDriveMotorID, DriveConstants.frontRightDriveMotorID, DriveConstants.backLeftDriveMotorID, DriveConstants.backRightDriveMotorID);
        } else {
            driveHardware = new DriveTalon(DriveConstants.frontLeftDriveMotorID, DriveConstants.frontRightDriveMotorID, DriveConstants.backLeftDriveMotorID, DriveConstants.backRightDriveMotorID);
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
