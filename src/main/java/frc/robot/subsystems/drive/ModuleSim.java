package frc.robot.subsystems.drive;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;


public class ModuleSim extends DriveIO {

    private final DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.driveWheelGearReduction, 1);
    private final DCMotorSim rotationMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1); // TODO: Change these

    private final PIDController drivePID = new PIDController(0, 0, 0);
    private final PIDController rotationPID = new PIDController(DriveConstants.rotationKp, 0, DriveConstants.rotationKd);

    private SwerveModulePosition modulePosition = new SwerveModulePosition();
    private SwerveModuleState goalModuleState =  new SwerveModuleState();

    public ModuleSim() {
        drivePID.setP(DriveConstants.driveKp); 
        drivePID.setD(DriveConstants.driveKd);

        rotationPID.enableContinuousInput(-Math.PI, Math.PI);    
    }

    public double getDrivePosition() {
        return driveMotor.getAngularPositionRad();
    }

    public double getRotationPosition() {
        return rotationMotor.getAngularPositionRad();
    }

    public double getRotationVelocity() {
        return rotationMotor.getAngularVelocityRadPerSec();
    }

    public double getDriveVelocityMPerS() {
        return driveMotor.getAngularVelocityRadPerSec() * DriveConstants.wheelRadiusM;
    }       

    public double getDriveVelocityRadPerS() {
        return driveMotor.getAngularVelocityRadPerSec(); 
    }

    public void setRotationVoltage(double volts) {
        rotationMotor.setInputVoltage(volts);
    }

    public void setDriveVoltage(double volts) {
        driveMotor.setInputVoltage(volts);
    }

    public void setDriveVelocity(double velocityRadPerS) {
        double volt = drivePID.calculate(getDriveVelocityRadPerS(), velocityRadPerS);
        driveMotor.setInputVoltage(volt);

    }

    public void zeroEncoders() {}

    public void setDrivePD(double p, double d) {
        drivePID.setP(p);
        drivePID.setD(d);
    }

    public double getDriveStatorCurrent() {
        return driveMotor.getCurrentDrawAmps();
    }

    public double getRotationStatorCurrent() {
        return rotationMotor.getCurrentDrawAmps();
    }

    public void setBrake(boolean braked) {}

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

    public void updateModuleSimulation() {
        driveMotor.update(0.02);
        rotationMotor.update(0.02);
    }

    @Override
    public void periodic() {
        updateModuleSimulation();
    }
}
