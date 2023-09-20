package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ArmSubsystem extends SubsystemBase{
    
    private CANSparkMax wristMotor;
    private double wristkp = 0, wristki = 0, wristkd = 0;
    private double wristTolerance = 0.01;
    private PIDController wristController = new PIDController(wristkp, wristki, wristkd);
    private static double wristGoalRad = 0;
    private final double softStop = 80.0, hardStop = 100.0;

    private CANSparkMax leftMotor, rightMotor, intakeMotor;
    private double shooterGoalPower;
    private double intakeGoalPower;

    private double wristOffset;


    private double wristIntake, wristShootHigh, wristShootMid, wristShootLow, wristIdle;
    private double shooterIntake, shooterShootHigh, shooterShootMid, shooterShootLow, shooterIdle = 0;
    private double intakeOn, intakeOff = 0;

    public static enum ArmPositions {
        Intake,
        ShootHigh,
        ShootMid,
        ShootLow,
        Idle,
        Home;
    }

    private ArmPositions currentArmPos = ArmPositions.Idle;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.ArmConstants.intakeMotorID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
        leftMotor.follow(rightMotor, true);
        wristOffset = 0.0;

    }
    
    public void setMode(ArmPositions armPos) {
        currentArmPos = armPos;

        if (armPos == ArmPositions.Intake) {
            wristGoalRad = wristIntake;
            shooterGoalPower = shooterIntake;
            intakeGoalPower = intakeOn;
        } else if (armPos == ArmPositions.ShootHigh) {
            wristGoalRad = wristShootHigh;
            shooterGoalPower = shooterShootHigh;
            intakeGoalPower = intakeOff;
        } else if (armPos == ArmPositions.ShootMid) {
            wristGoalRad = wristShootMid;
            shooterGoalPower = shooterShootMid;
            intakeGoalPower = intakeOff;
        } else if (armPos == ArmPositions.ShootLow) {
            wristGoalRad = wristShootLow;
            shooterGoalPower = shooterShootLow;
            intakeGoalPower = intakeOff;
        } else if (armPos == ArmPositions.Idle) {
            wristGoalRad = wristIdle;
            shooterGoalPower = shooterIdle;
            intakeGoalPower = intakeOff;
        } else if(armPos == ArmPositions.Home){
            shooterGoalPower = shooterIdle;
            intakeGoalPower = intakeOff;
            wristGoalRad = wristShootLow;
            if(getWristMotorAmps() > 10.0){
                wristGoalRad = getWristPositionRad();
                wristOffset = getWristPositionRad(); 
            }
        }
    }

    public ArmPositions getArmPos() {
        return currentArmPos;
    }
  
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    public double getWristPositionRad() {
        return wristMotor.getEncoder().getPosition() * 2 * Math.PI - wristOffset;
    }

    public void setWristMotorPower(double percent) {
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

    ///////////////////////////////////////////////////////////////////////////////////////////
    

    public void setShooterMotorPower(double percent) {
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


    public void shooterControl() {
        setShooterMotorPower(shooterGoalPower);
        checkShooterAmps();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
   
    public void setIntakeMotorPower(double percent) {
        intakeMotor.set(percent);
    }

    public double getIntakeMotorAmps() {
        return intakeMotor.getOutputCurrent();
    } 

    private void checkIntakeAmps() {
        if (getIntakeMotorAmps() > hardStop) {
            intakeGoalPower = 0;
            setIntakeMotorPower(0);
 
        }
    }

    public void setIntakeGoalPower(double power) {
        intakeGoalPower = power;
    }

    public void intakeControl() {
        setIntakeGoalPower(intakeGoalPower);
        checkIntakeAmps();
    }



    @Override
    public void periodic() {

        SmartDashboard.putNumber("wrist position radians", getWristPositionRad());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());
        wristkp = SmartDashboard.getNumber("wrist kp", 0);
        wristki = SmartDashboard.getNumber("wrist ki", 0);
        wristkd = SmartDashboard.getNumber("wrist kd", 0);

        wristIntake = SmartDashboard.getNumber("wrist intake rad", 0);
        wristShootHigh = SmartDashboard.getNumber("wrist shoot high rad", 0);
        wristShootMid = SmartDashboard.getNumber("wrist shoot mid rad", 0);
        wristShootLow = SmartDashboard.getNumber("wrist shoot low rad", 0);
        wristIdle = SmartDashboard.getNumber("wrist shoot idle rad", 0);

        shooterIntake = SmartDashboard.getNumber("shooter intake power", 0);
        shooterShootHigh = SmartDashboard.getNumber("shooter shoot high power", 0);
        shooterShootMid = SmartDashboard.getNumber("shooter shoot mid power", 0);
        shooterShootLow = SmartDashboard.getNumber("shooter shoot low power", 0);
        shooterIdle = SmartDashboard.getNumber("shooter idle power", 0);

        intakeOn = SmartDashboard.getNumber("intake power", 0);

        if(getArmPos() == ArmPositions.Home){

        }

        wristControl();   
        
        if(wristFinished()) {
            shooterControl();
            intakeControl();
        }
    }


}