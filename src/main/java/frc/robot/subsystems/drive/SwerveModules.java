package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class SwerveModules extends SubsystemBase{

    private final CANSparkMax driveMotor;
    private final CANSparkMax rotationMotor;

    private final PIDController drivePID = new PIDController(0, 0, 0);
    private final PIDController rotationPID = new PIDController(DriveConstants.rotationKp, 0, DriveConstants.rotationKd);

    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModuleState goalModuleState =  new SwerveModuleState();

    //private final CANCoder driveEncoder;  it is built in
    private final CANCoder rotationEncoder;

    private final double initialOffsetRadians;

    public SwerveModules(int driveMotorID, int rotationMotorID, int cancoderID, double measuredOffsetsRadians, boolean driveModeReversed, boolean turningMotorReversed 
        ){

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        rotationMotor = new CANSparkMax(rotationMotorID, MotorType.kBrushless);
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

        // drivePidController = driveMotor.getPIDController();
        // drivePidController.setFeedbackDevice(driveMotor.getEncoder());

        drivePID.setP(DriveConstants.driveKp); 
        drivePID.setD(DriveConstants.driveKd);

        //drivePidController.setOutputRange(0, 100);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        
    
    }

    public double getDrivePosition() {
        return driveMotor.getEncoder().getPosition() * 2.0 * Math.PI / DriveConstants.driveWheelGearReduction;
    }       //number of rotations to radians per second

    public double getRotationPosition() {
        return MathUtil.angleModulus(Units.degreesToRadians(rotationEncoder.getPosition())) - initialOffsetRadians;
    }

    public double getRotationVelocity() {
        return Units.degreesToRadians(rotationEncoder.getVelocity());
    }

    /* Velocity gives Rotations per minute, need to go to radians per seconds
     * 
    */
    public double getDriveVelocityMPerS() {
        return driveMotor.getEncoder().getVelocity() * 2 * Math.PI * DriveConstants.wheelRadiusM / DriveConstants.driveWheelGearReduction;
    }       

    public double getDriveVelocityRadPerS() {
        return driveMotor.getEncoder().getVelocity() * 2 * Math.PI; 
    }

    public void setRotationVoltage(double volts) {
        rotationMotor.set(volts/12);
    }

    public void setDriveVoltage(double volts) {
        driveMotor.set(volts/12);
    }

    public void setDriveVelocity(double velocityRadPerS) {
        double volt = drivePID.calculate(getDriveVelocityRadPerS(), velocityRadPerS);
        driveMotor.set(volt);

    }

    public void zeroEncoders() {
        rotationEncoder.setPositionToAbsolute(1000);
        driveMotor.getEncoder().setPosition(0);
    }

    public void setDrivePD(double p, double d) {
        drivePID.setP(p);
        drivePID.setD(d);
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

    public SwerveModulePosition getModulePosition(){
        return modulePosition;
    } 

    public SwerveModuleState getModuleState(){
        return goalModuleState;
    }

    public void setGoalModuleState(SwerveModuleState state){
        goalModuleState = state;
    }

    public void initModulePosition(){
        modulePosition.distanceMeters = getDrivePosition() * DriveConstants.wheelRadiusM;
        modulePosition.angle = new Rotation2d(getRotationPosition());

    }

    public void moduleControl(SwerveModuleState optimizedState){
        double rotationSetpointRadians = optimizedState.angle.getRadians();
        double speedSetpointMPerS = optimizedState.speedMetersPerSecond;

        //Set module speed
        double speedRadPerS = speedSetpointMPerS / DriveConstants.wheelRadiusM;

        this.setDriveVelocity(speedRadPerS);

        //Set module rotation
        double rotationVoltage = rotationPID.calculate(getRotationPosition(), rotationSetpointRadians);
        this.setRotationVoltage(rotationVoltage);

    }
}

