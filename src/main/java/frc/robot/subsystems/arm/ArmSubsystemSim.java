package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.Intake;

public class ArmSubsystemSim implements ArmSubsystemIO {

    private CANSparkMax wristMotor;
    private double wristTolerance = 1;
    private PIDController wristController = new PIDController(0, 0, 0);
    private double wristkP = 0.03;
    private double wristkI = 0.0;
    private double wristkD = 0.0;

    private final double softStop = 20.0;
    private final double hardStop = 25.0;

    private Intake intake;

    // SmartDashboard setpoint values
    private double wristIntakePosition;
    private double wristShootPosition;

    // SmartDashboard 
    private double shooterIntakePower = 0.5;
    private double shooterShootPower = -1;
    private double shooterWarmup = 0;

    private Mode currentMode = Mode.IDLE;
    private double wristGoalPosition = 0.0;
    
}
