package frc.robot.subsystems.arm;

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


    public static enum ArmPositions {

        Intake(0,0),
        ShootHigh(0,0),
        ShootMid(0,0),
        ShootLow(0,0),
        Idle(0,0);

        public final int wristPos, shooterPower;

        private ArmPositions(int wristPos, int shooterPower) {
            this.wristPos = wristPos;
            this.shooterPower = shooterPower;
        }
    }

    private ArmPositions currentArmPos = ArmPositions.Idle;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(Constants.ArmConstants.intakeMotorID, MotorType.kBrushless);
        leftMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
        leftMotor.follow(rightMotor, true);

    }
<<<<<<< HEAD
    
    public void setMode(ArmPositions armPos) {
        currentArmPos = armPos;
        wristGoalRad = armPos.wristPos;
        shooterGoalPower = armPos.shooterPower;
    }

    public ArmPositions getArmPos() {
        return currentArmPos;
    }
=======

    ////////////////////////////////////////////////////////////////////////////////////////////////////
>>>>>>> 3b562d1 (intake)

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


    public void shooterControl() {
        setShooterMotorPower(shooterGoalPower);
        checkShooterAmps();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
   
    private void setIntakeMotorPower(double percent) {
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
        setShooterMotorPower(intakeGoalPower);
        checkShooterAmps();
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
            shooterControl();
        }
    }


}