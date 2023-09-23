package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class BasicDriveSubsystem extends SubsystemBase {
    
    private MotorControllerGroup leftWheels;
    private MotorControllerGroup rightWheels;

    private DifferentialDrive drivetrain;

    public BasicDriveSubsystem() {
        CANSparkMax backLeft = new CANSparkMax(DriveConstants.backLeftID, MotorType.kBrushless);
        CANSparkMax frontLeft = new CANSparkMax(DriveConstants.frontLeftID, MotorType.kBrushless);

        CANSparkMax backRight = new CANSparkMax(DriveConstants.backRightID, MotorType.kBrushless);
        CANSparkMax frontRight = new CANSparkMax(DriveConstants.frontRightID, MotorType.kBrushless);

        backLeft.setInverted(true);
        frontLeft.setInverted(true);
        backRight.setInverted(false);
        frontRight.setInverted(false);

        leftWheels = new MotorControllerGroup(backLeft, frontLeft);
        rightWheels = new MotorControllerGroup(backRight, frontRight);

        drivetrain = new DifferentialDrive(leftWheels, rightWheels);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }
}
