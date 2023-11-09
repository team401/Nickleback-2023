package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModuleIO extends SubsystemBase{
    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final PIDController drivePID = new PIDController(0, 0, 0);
    private final PIDController rotationPID = new PIDController(DriveConstants.rotationKp, 0, DriveConstants.rotationKd);

    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModuleState goalModuleState =  new SwerveModuleState();

    //private final CANCoder driveEncoder;  it is built in
    private final CANCoder rotationEncoder;


    
}
