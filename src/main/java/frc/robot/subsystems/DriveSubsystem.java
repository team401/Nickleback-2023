package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{

    public enum ChassisMode {
        B_TEAM,
        C_TEAM
    }

    public enum DriveMode {
        TANKDRIVE,
        ARCADEDRIVE
    }
    private DriveMode mode;

    private DriveHardware driveHardware;

    private DriveMode mode;
    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;


    public DriveSubsystem(ChassisMode chassisMode, DriveMode driveMode) {
        if (chassisMode == ChassisMode.B_TEAM) {
            driveHardware = new DriveSparkMAX(1, 2, 3, 4);
        } else {
            driveHardware = new DriveTalon(1, 2, 3, 4);
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
