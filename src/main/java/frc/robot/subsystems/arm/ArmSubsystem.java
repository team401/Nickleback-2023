package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ArmSubsystem extends SubsystemBase{
    
    private CANSparkMax wristMotor;
    private double wristkp = 0, wristki = 0, wristkd = 0;
    private double wristTolerance = 0.01;
    private PIDController wristController = new PIDController(wristkp, wristki, wristkd);
    private static double wristGoalRad = 0;
    private final double softStop = 80.0, hardStop = 100.0;

    private CANSparkMax leftMotor, rightMotor;
    private double shooterGoalPower;


    public ArmSubsystem() {
        wristMotor = new CANSparkMax(0, MotorType.kBrushless);

        leftMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightMotor = new CANSparkMax(2, MotorType.kBrushless);
        leftMotor.follow(rightMotor, true);

    }

    public double getWristPositionRad() {
        return wristMotor.getEncoder().getPosition() * 2 * Math.PI;
    }

    private void setWristMotorPower(double percent) {
        wristMotor.set(percent);
    }

    public double getWristMotorAmps() {
        return wristMotor.getOutputCurrent();
    }

    private void checkWristAmps() {
        if (getWristMotorAmps() > softStop) {
            wristGoalRad = getWristPositionRad();
        } 
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
        }
    }

    public void setWristGoalRads(double rad) {
        wristGoalRad = rad;
    }

    private void wristControl() {
        // add TrapezoidProfile + feedforward later?
        double start = getWristPositionRad();
        double x = wristController.calculate(wristGoalRad, start);
        setWristMotorPower(x);
        checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPositionRad()-wristGoalRad) < wristTolerance;
    }
    

    private void setShooterMotorPower(double percent) {
        rightMotor.set(percent);
    }

    public double getShooterMotorAmps() {
        return rightMotor.getOutputCurrent();
    } 

    private void checkShooterAmps() {
        if (getShooterMotorAmps() > hardStop) {
            shooterGoalPower = 0;
            setShooterMotorPower(0);
        }
    }

    public void setShooterGoalPower(double power) {
        shooterGoalPower = power;
    }


    @Override
    public void periodic() {
        wristControl();   

        SmartDashboard.putNumber("wrist position radians", getWristPositionRad());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());
        wristkp = SmartDashboard.getNumber("wrist kp", 0);
        wristki = SmartDashboard.getNumber("wrist ki", 0);
        wristkd = SmartDashboard.getNumber("wrist kd", 0);

        if(wristFinished()) {
            setShooterMotorPower(shooterGoalPower);
            checkShooterAmps();
        }
    }


}