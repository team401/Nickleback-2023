package frc.robot.subsystems;

import java.util.concurrent.ThreadPoolExecutor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class ArmSubsystem extends SubsystemBase{
    
    private CANSparkMax wristMotor;
    private double wristkp = 0, wristki = 0, wristkd = 0, wristkg = 0, wristkv = 0, wristks = 0;
    private final double wristTolerance = 0.01;
    private PIDController wristController = new PIDController(wristkp, wristki, wristkd);
    private ArmFeedforward feedforward = new ArmFeedforward(wristks, wristkg, wristkv);
    private static double wristGoalRad = 0;

    private final double softStop = 30.0, hardStop = 40.0;
    private final double voltageLimit = 12;

    private CANSparkMax leftMotor, rightMotor, intakeMotor;
    private double shooterGoalPower;
    private double intakeGoalPower;

    private double wristIntake, wristShootHigh, wristShootMid, wristShootLow, wristWarmup;
    private double shooterIntake, shooterShootHigh, shooterShootMid, shooterShootLow, shooterWarmup = 0;
    private double intakeOn, intakeOff = 0;

    public static enum ArmPositions {
        Intake,
        ShootHigh,
        ShootMid,
        ShootLow,
        Warmup,
        Idle;
    }

    private ArmPositions currentArmPos;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.ArmConstants.intakeMotorID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
        leftMotor.follow(rightMotor, true);
        wristMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        setMode(ArmPositions.Warmup);
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
        } else if (armPos == ArmPositions.Warmup) {
            wristGoalRad = wristWarmup;
            shooterGoalPower = shooterWarmup;
            intakeGoalPower = intakeOff;
        }
        
    }

    public ArmPositions getArmPos() {
        return currentArmPos;
    }
  
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    public double getWristPositionRad() {
        return wristMotor.getEncoder().getPosition() * 2 * Math.PI / 10; 
        // ???? check gear ratios again because I don't know if I calculated them right
        // from Abby: 10:1, 3:1, 16:48
    }

    public void setWristMotorPower(double percent) {
        if (12 * Math.abs(percent) < voltageLimit) wristMotor.set(percent);
        else wristMotor.set(voltageLimit/12 * Math.abs(percent)/percent);
    }

    public double getWristMotorAmps() {
        return wristMotor.getOutputCurrent();
    }

    private void checkWristAmps() {
        if (getWristMotorAmps() > softStop) {
            wristGoalRad = getWristPositionRad();
            if (getWristMotorAmps() > hardStop) {
                setWristMotorPower(0);
                currentArmPos = ArmPositions.Idle;
            }
        } 
        
    }

    private void wristControl() {
        // add TrapezoidProfile + feedforward later?
        if (currentArmPos != ArmPositions.Idle) {
            double start = getWristPositionRad();
            double x = wristController.calculate(wristGoalRad, start) + feedforward.calculate(wristGoalRad, 0);
            setWristMotorPower(x);
            checkWristAmps();
        } else {
            setWristMotorPower(0);
        }
    }

    public boolean wristFinished() {
        return Math.abs(getWristPositionRad()-wristGoalRad) < wristTolerance;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////
    

    public void setShooterMotorPower(double percent) {
        if (12 * Math.abs(percent) < voltageLimit) rightMotor.set(percent);
        else rightMotor.set(voltageLimit/12 * Math.abs(percent)/percent);

    }

    public double getShooterMotorAmps() {
        return rightMotor.getOutputCurrent();
    } 

    private void checkShooterAmps() {
        if (getShooterMotorAmps() > hardStop) {
            shooterGoalPower = 0;
            setShooterMotorPower(0);
            currentArmPos = ArmPositions.Idle;
        }
    }

    public void shooterControl() {
        if (currentArmPos != ArmPositions.Idle) {
            setShooterMotorPower(shooterGoalPower);
            checkShooterAmps();
        } else {
            setShooterMotorPower(0);
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
   
    public void setIntakeMotorPower(double percent) {
        if (12 * Math.abs(percent) < voltageLimit) intakeMotor.set(percent);
        else intakeMotor.set(voltageLimit/12 * Math.abs(percent)/percent);
    }

    public double getIntakeMotorAmps() {
        return intakeMotor.getOutputCurrent();
    } 

    private void checkIntakeAmps() {
        if (getIntakeMotorAmps() > hardStop) {
            intakeGoalPower = 0;
            setIntakeMotorPower(0);
            currentArmPos = ArmPositions.Idle;
        }
    }

    public void setIntakeGoalPower(double power) {
        intakeGoalPower = power;
    }

    public void intakeControl() {
        if (currentArmPos != ArmPositions.Idle) {
            setIntakeGoalPower(intakeGoalPower);
            checkIntakeAmps();
        } else {
            setIntakeGoalPower(0);
        }
    }



    @Override
    public void periodic() {

        SmartDashboard.putNumber("wrist position radians", getWristPositionRad());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());
        wristkp = SmartDashboard.getNumber("wrist kp", 0);
        wristki = SmartDashboard.getNumber("wrist ki", 0);
        wristkd = SmartDashboard.getNumber("wrist kd", 0);
        wristks = SmartDashboard.getNumber("wrist ks", 0);
        wristkg = SmartDashboard.getNumber("wrist kg", 0);
        wristkv = SmartDashboard.getNumber("wrist kv", 0);

        SmartDashboard.putNumber("shooter amps", getShooterMotorAmps());
        SmartDashboard.putNumber("intake amps", getIntakeMotorAmps());

        wristIntake = SmartDashboard.getNumber("wrist intake rad", 0);
        wristShootHigh = SmartDashboard.getNumber("wrist shoot high rad", 0);
        wristShootMid = SmartDashboard.getNumber("wrist shoot mid rad", 0);
        wristShootLow = SmartDashboard.getNumber("wrist shoot low rad", 0);
        wristWarmup = SmartDashboard.getNumber("wrist shoot idle rad", 0);

        shooterIntake = SmartDashboard.getNumber("shooter intake power", 0);
        shooterShootHigh = SmartDashboard.getNumber("shooter shoot high power", 0);
        shooterShootMid = SmartDashboard.getNumber("shooter shoot mid power", 0);
        shooterShootLow = SmartDashboard.getNumber("shooter shoot low power", 0);
        shooterWarmup = SmartDashboard.getNumber("shooter idle power", 0);

        intakeOn = SmartDashboard.getNumber("intake power", 0);

        
        wristControl();   
        
        if(wristFinished()) {
            shooterControl();
            intakeControl();
        }
    }


}