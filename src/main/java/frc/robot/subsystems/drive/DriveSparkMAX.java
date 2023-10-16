package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSparkMAX{

    private final CANSparkMax driveMotorFrontLeft;
    private final CANSparkMax driveMotorFrontRight;
    private final CANSparkMax driveMotorBackLeft;
    private final CANSparkMax driveMotorBackRight;

    public DriveSparkMAX(int frontLeftDriveMotorID, int frontRightDriveMotorID, int backLeftDriveMotorID, int backRightDriveMotorID) {
        
        driveMotorFrontLeft = new CANSparkMax(frontLeftDriveMotorID, MotorType.kBrushless);
        driveMotorFrontRight = new CANSparkMax(frontRightDriveMotorID, MotorType.kBrushless);
        driveMotorBackLeft = new CANSparkMax(backLeftDriveMotorID, MotorType.kBrushless);
        driveMotorBackRight = new CANSparkMax(backRightDriveMotorID, MotorType.kBrushless);

        driveMotorFrontLeft.setIdleMode(IdleMode.kCoast);
        driveMotorFrontRight.setIdleMode(IdleMode.kCoast);
        driveMotorBackLeft.setIdleMode(IdleMode.kCoast);
        driveMotorBackRight.setIdleMode(IdleMode.kCoast);
        
    }

}
