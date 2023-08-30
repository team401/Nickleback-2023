package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class WristSubsystem extends SubsystemBase{
    
    private CANSparkMax wristMotor;
    private double kp = 0, ki = 0, kd = 0;
    private double tolerance = 0.01;
    private PIDController controller = new PIDController(kp, ki, kd);
    private static double goalRad = 0;

    public WristSubsystem() {
        wristMotor = new CANSparkMax(0, MotorType.kBrushless);

    }

    public double getPositionRad() {
        return wristMotor.getEncoder().getPosition() * 2 * Math.PI;
    }

    public void setMotorPower(double percent) {
        wristMotor.set(percent);
    }

    public double getMotorAmps() {
        return wristMotor.getOutputCurrent();
    }

    private void checkAmps() {
        if (getMotorAmps() > 80.0) {
            goalRad = getPositionRad();
        } 
        if (getMotorAmps() > 100.0) {
            setMotorPower(0);
        }
    }

    public void setGoalRads(double rad) {
        goalRad = rad;
    }

    public void control() {
        // add TrapezoidProfile + feedforward later?
        double start = getPositionRad();
        double x = controller.calculate(goalRad, start);
        setMotorPower(x);
        checkAmps();
    }

    public boolean finished() {
        return getPositionRad() > goalRad - tolerance && getPositionRad() < goalRad + tolerance;
    }
    
    @Override
    public void periodic() {
        control();   

        SmartDashboard.putNumber("wrist position radians", getPositionRad());
        SmartDashboard.putNumber("wrist amps", getMotorAmps());
        SmartDashboard.putBoolean("wrist finished", finished());
        kp = SmartDashboard.getNumber("wrist kp", 0);
        ki = SmartDashboard.getNumber("wrist ki", 0);
        kd = SmartDashboard.getNumber("wrist kd", 0);
        
    }


}