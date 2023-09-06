package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase{

    private final WPI_TalonFX driveMotorFrontLeft;
    private final WPI_TalonFX driveMotorFrontRight;
    private final WPI_TalonFX driveMotorBackLeft;
    private final WPI_TalonFX driveMotorBackRight;
    private final MotorControllerGroup rightMotorControllerGroup;
    private final MotorControllerGroup leftMotorControllerGroup;
    private final DifferentialDrive drive;
	private final PigeonIMU pigeon;
    private final double angleOffset = 0;
    private enum DriveMode{
        TANKDRIVE,
        ARCADEDRIVE
    }

    private DriveMode mode;
    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;

    public DriveSubsystem(){
        driveMotorFrontLeft = new WPI_TalonFX(Constants.DriveConstants.frontLeftDriveMotorID);
        driveMotorFrontRight = new WPI_TalonFX(Constants.DriveConstants.frontRightDriveMotorID);
        driveMotorBackLeft = new WPI_TalonFX(Constants.DriveConstants.backLeftDriveMotorID);
        driveMotorBackRight = new WPI_TalonFX(Constants.DriveConstants.backRightDriveMotorID);
        driveMotorFrontLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorFrontRight.setNeutralMode(NeutralMode.Coast);
        driveMotorBackLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorBackRight.setNeutralMode(NeutralMode.Coast);
        rightMotorControllerGroup = new MotorControllerGroup(driveMotorFrontRight, driveMotorBackRight);
        leftMotorControllerGroup = new MotorControllerGroup(driveMotorFrontLeft, driveMotorBackLeft);
        drive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);
        pigeon = new PigeonIMU(DriveConstants.pigeonID);
    }

    public double GetDriveAngle()
	{
		return pigeon.getYaw()-angleOffset;
	}


    public void setArcadeDriveControls(double forward, double rotation) {
		arcadeForward = forward;
        arcadeRotation = rotation;
	}

    public void arcadeDrive(){
        drive.arcadeDrive(arcadeForward, -arcadeRotation);
    }

	public void setTankDriveControls(double left, double right) {
        leftTank = left;
        rightTank = right;
	}

    public void tankDrive(){
        drive.tankDrive(leftTank, rightTank);
    }

    @Override
    public void periodic(){
        if(mode == DriveMode.TANKDRIVE){
            tankDrive();
        }
        else{
            arcadeDrive();
        }
    } 

}
