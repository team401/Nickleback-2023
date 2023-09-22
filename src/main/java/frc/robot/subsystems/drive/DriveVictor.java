package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DriveVictor extends DriveHardware {

    private final WPI_VictorSPX driveMotorFrontLeft;
    private final WPI_VictorSPX driveMotorFrontRight;
    private final WPI_VictorSPX driveMotorBackLeft;
    private final WPI_VictorSPX driveMotorBackRight;

    
    public DriveVictor(int frontLeftDriveMotorID, int frontRightDriveMotorID, int backLeftDriveMotorID, int backRightDriveMotorID) {
        
        driveMotorFrontLeft = new WPI_VictorSPX(frontLeftDriveMotorID);
        driveMotorFrontRight = new WPI_VictorSPX(frontRightDriveMotorID);
        driveMotorBackLeft = new WPI_VictorSPX(backLeftDriveMotorID);
        driveMotorBackRight = new WPI_VictorSPX(backRightDriveMotorID);

        driveMotorFrontLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorFrontRight.setNeutralMode(NeutralMode.Coast);
        driveMotorBackLeft.setNeutralMode(NeutralMode.Coast);
        driveMotorBackRight.setNeutralMode(NeutralMode.Coast);
        
        super.rightMotorControllerGroup = new MotorControllerGroup(driveMotorFrontRight, driveMotorBackRight);
        super.leftMotorControllerGroup = new MotorControllerGroup(driveMotorFrontLeft, driveMotorBackLeft);

        super.drive = new DifferentialDrive(super.leftMotorControllerGroup, super.rightMotorControllerGroup);
        
    }
    
}
