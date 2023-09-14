package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveTalon extends DriveHardware {

    private final WPI_TalonFX driveMotorFrontLeft;
    private final WPI_TalonFX driveMotorFrontRight;
    private final WPI_TalonFX driveMotorBackLeft;
    private final WPI_TalonFX driveMotorBackRight;

    
    public DriveTalon(int frontLeftDriveMotorID, int frontRightDriveMotorID, int backLeftDriveMotorID, int backRightDriveMotorID) {
        
        driveMotorFrontLeft = new WPI_TalonFX(frontLeftDriveMotorID);
        driveMotorFrontRight = new WPI_TalonFX(frontRightDriveMotorID);
        driveMotorBackLeft = new WPI_TalonFX(backLeftDriveMotorID);
        driveMotorBackRight = new WPI_TalonFX(backRightDriveMotorID);

        driveMotorFrontLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorFrontRight.setNeutralMode(NeutralMode.Coast);
        driveMotorBackLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorBackRight.setNeutralMode(NeutralMode.Coast);
        
        super.rightMotorControllerGroup = new MotorControllerGroup(driveMotorFrontRight, driveMotorBackRight);
        super.leftMotorControllerGroup = new MotorControllerGroup(driveMotorFrontLeft, driveMotorBackLeft);

        super.drive = new DifferentialDrive(super.leftMotorControllerGroup, super.rightMotorControllerGroup);
        
    }
    
}
