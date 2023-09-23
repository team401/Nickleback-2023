package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;



public class ArmSubsystem extends SubsystemBase{
    
    private CANSparkMax wristMotor;
    private double wristTolerance = 0.01;
    private PIDController wristController = new PIDController(ArmConstants.wristkP, 0, ArmConstants.wristkD);
    private static double wristGoalRad = 0;
    private final double softStop = 80.0, hardStop = 100.0;

    private CANSparkMax leftIntakeMotor;
    private CANSparkMax rightIntakeMotor;

    private double shooterGoalPower;
    private double intakeGoalPower;


    public static enum ArmPositions {

        Intake(0,0,0),
        ShootHigh(0,0,0),
        ShootMid(0,0,0),
        ShootLow(0,0,0),
        Idle(0,0,0);

        public final int wristPos, shooterPower, intakePower;

        private ArmPositions(int wristPos, int shooterPower, int intakePower) {
            this.wristPos = wristPos;
            this.shooterPower = shooterPower;
            this.intakePower = intakePower;
        }
    }

    private ArmPositions currentArmPos = ArmPositions.Idle;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
    
        leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);

        rightIntakeMotor.follow(leftIntakeMotor, true);
    }

    
    public void setMode(ArmPositions armPos) {
        currentArmPos = armPos;
        wristGoalRad = armPos.wristPos;
        shooterGoalPower = armPos.shooterPower;
        intakeGoalPower = armPos.intakePower;
    }

    public ArmPositions getArmPos() {
        return currentArmPos;
    }
  
    ////////////////////////////////////////////////////////////////////////////////////////////////////

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


    public void wristControl() {
        // add TrapezoidProfile + feedforward later?
        double start = getWristPositionRad();
        double x = wristController.calculate(wristGoalRad, start);
        setWristMotorPower(x);
        checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPositionRad()-wristGoalRad) < wristTolerance;
    }
   
    public void setIntakeMotorPower(double percent) {
        leftIntakeMotor.set(percent);
    }

    public void intake() {
        leftIntakeMotor.set(0.2);
    }

    public void shoot() {
        leftIntakeMotor.set(-0.6);
    }

    public void stopIntake() {
        leftIntakeMotor.set(0.0);
    }

    public double getIntakeMotorAmps() {
        return leftIntakeMotor.getOutputCurrent();
    } 

    private void checkIntakeAmps() {
        if (getIntakeMotorAmps() > hardStop) {
            intakeGoalPower = 0;
            setIntakeMotorPower(0);
        }
    }

    @Override
    public void periodic() {
        // wristControl();

        SmartDashboard.putNumber("wrist position radians", getWristPositionRad());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());
        // wristkp = SmartDashboard.getNumber("wrist kp", 0);
        // wristki = SmartDashboard.getNumber("wrist ki", 0);
        // wristkd = SmartDashboard.getNumber("wrist kd", 0);

        checkIntakeAmps();
    }


}