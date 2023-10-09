package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class SwerveModules  extends SubsystemBase{

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final SparkMaxPIDController drivePidController;

    //private final CANCoder driveEncoder;  it is built in
    private final CANCoder rotationEncoder;

    private final double initialOffsetRadians;

    public SwerveModules(int driveModeID, int turningMotorID, int cancoderID, double measuredOffsetsRadians, boolean driveModeReversed, boolean turningMotorReversed 
        ){

        driveMotor = new CANSparkMax(driveModeID, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);
        rotationEncoder = new CANCoder(cancoderID);

        driveMotor.setIdleMode(IdleMode.kCoast);
        rotationMotor.setIdleMode(IdleMode.kCoast);

        driveMotor.setInverted(driveModeReversed);
        rotationMotor.setInverted(turningMotorReversed);

        driveMotor.setSmartCurrentLimit(70, 80, 1);
        rotationMotor.setSmartCurrentLimit(70, 80, 1);

        rotationEncoder.configFactoryDefault(1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20, 1000);
        rotationEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 255, 1000);

        initialOffsetRadians = measuredOffsetsRadians;

        drivePidController = driveMotor.getPIDController();
        drivePidController.setFeedbackDevice(driveMotor.getEncoder());

        drivePidController.setP(DriveConstants.driveKps[1]);
        drivePidController.setD(DriveConstants.driveKds[1]);

        drivePidController.setOutputRange(0, 100);
    
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition() / 2048.0 * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
    }       //number of rotations to radians per second

    public double getRotationPosition() {
        return MathUtil.angleModulus(Units.degreesToRadians(rotationEncoder.getPosition())) - initialOffsetRadians;
    }

    public double getRotationVelocity() {
        return Units.degreesToRadians(rotationEncoder.getVelocity());
    }

    public double getDriveVelocityMPerS() {
        return driveMotor.getEncoder().getVelocity() / 2048.0 * 10.0 * 2 * Math.PI * DriveConstants.wheelRadiusM / DriveConstants.driveWheelGearReduction;
    }       //veclocity gives Rotations per minute, need to go to radians per second

    public void setRotationVoltage(double volts) {
        rotationMotor.set(volts/12);
    }

    public void setDriveVoltage(double volts) {
        driveMotor.set(volts/12);
    }

    public void setDriveVelocity(double velocityRadPerS, double ffVolts) {
        //double velocityTicksPer100ms = velocityRadPerS * 2048.0 / 10.0 / 2.0 / Math.PI * DriveConstants.driveWheelGearReduction;
        driveMotor.set(ffVolts / 12.0);
    }


    public void zeroEncoders() {
        rotationEncoder.setPositionToAbsolute(1000);
        driveMotor.getEncoder().setPosition(0);
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocityMPerS(), new Rotation2d(getRotationPosition()));
    }

    public void setDrivePD(double p, double d) {
        drivePidController.setP(p);
        drivePidController.setD(d);
    }

    public double getDriveStatorCurrent() {
        return driveMotor.getOutputCurrent();
    }

    public double getRotationStatorCurrent() {
        return rotationMotor.getOutputCurrent();
    }

    public void setBrake(boolean braked) {
        driveMotor.setIdleMode(braked ? IdleMode.kBrake : IdleMode.kCoast);
        rotationMotor.setIdleMode(braked ? IdleMode.kBrake : IdleMode.kCoast);
    }



    
}
