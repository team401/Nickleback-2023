package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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

public class NeoDriveSubsystem extends SubsystemBase{

    private final CANSparkMax leftMotor1;
    private final CANSparkMax leftMotor2;
    private final CANSparkMax rightMotor1;
    private final CANSparkMax rightMotor2;
    private final MotorControllerGroup rightMotorControllerGroup;
    private final MotorControllerGroup leftMotorControllerGroup;
    private final DifferentialDrive drive;
	// private final PigeonIMU pigeon;
    // private final double angleOffset = 0;
    private enum DriveMode{
        TANKDRIVE,
        ARCADEDRIVE
    }

    private DriveMode mode = DriveMode.TANKDRIVE;
    private double leftTank = 0;
    private double rightTank = 0;
    private double arcadeForward = 0;
    private double arcadeRotation = 0;

    public NeoDriveSubsystem(){
        leftMotor1 = new CANSparkMax(52, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(53, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(14, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(16, MotorType.kBrushless);
        
        leftMotorControllerGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
        rightMotorControllerGroup = new MotorControllerGroup(rightMotor1, rightMotor2);
        drive = new DifferentialDrive(leftMotorControllerGroup, rightMotorControllerGroup);
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
