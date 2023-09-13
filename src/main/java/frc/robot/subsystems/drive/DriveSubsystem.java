package frc.robot.subsystems.drive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.B_Configs;
import frc.robot.Constants.C_Configs;
import frc.robot.Constants.DriveMode;
import frc.robot.Constants.ChassisMode;;

public class DriveSubsystem extends SubsystemBase{


    private DriveHardware driveHardware;

    private DriveMode mode;
    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;

    public DriveSubsystem(ChassisMode chassisMode, DriveMode driveMode) {
        if (chassisMode == ChassisMode.B_TEAM) {
            driveHardware = new DriveSparkMAX(B_Configs.frontLeftDriveMotorID, B_Configs.frontRightDriveMotorID, B_Configs.backLeftDriveMotorID, B_Configs.backRightDriveMotorID);
        } else {
            driveHardware = new DriveTalon(C_Configs.frontLeftDriveMotorID, C_Configs.frontRightDriveMotorID, C_Configs.backLeftDriveMotorID, C_Configs.backRightDriveMotorID);
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
