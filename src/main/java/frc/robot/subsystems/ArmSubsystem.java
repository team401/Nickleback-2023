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
    private static double wristGoalPower = 0;
    private final double softStop = 20.0, hardStop = 25.0;

    private CANSparkMax leftIntakeMotor;
    private CANSparkMax rightIntakeMotor;

    private double shooterGoalPower;
    //private double intakeGoalPower;

    private double wristIntake, wristShoot, wristWarmup;
    private double shooterIntake, shooterShoot, shooterWarmup = 0;
    //private double intakeOn, intakeOff = 0;
    public double wristPos, shooterPower, intakePower;


    public static enum Modes {
        IDLE,
        INTAKE,
        WARMUP,
        SHOOT;
    }

    private Modes currentMode = Modes.IDLE;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
    
        leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);

        rightIntakeMotor.follow(leftIntakeMotor, true);
    }
    
    public void setMode(Modes mode) {
        currentMode = mode;
    }

    public void getSetPointFromMode (){
        if (this.currentMode == Modes.INTAKE) {
            wristGoalPower = wristIntake;
            shooterGoalPower = shooterIntake;
            //intakeGoalPower = intakeOn;
        } 
        else if (this.currentMode == Modes.SHOOT) {
            wristGoalPower = wristShoot;
            shooterGoalPower = shooterShoot;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Modes.WARMUP) {
            wristGoalPower = wristWarmup;
            shooterGoalPower = shooterWarmup;
            //intakeGoalPower = intakeOff;
        }
        else {
            //IDLE MODE
            wristGoalPower = 0;
        }
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
            wristGoalPower = getWristPositionRad();
        } 
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
            setMode(Modes.IDLE);
        }
    }


    public void wristControl() {
        // add TrapezoidProfile + feedforward later?
        //double start = getWristPositionRad();
        //double x = wristController.calculate(wristGoalPower, start);
        setWristMotorPower(wristGoalPower);
        checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPositionRad()-wristGoalPower) < wristTolerance;
    }
   
    public void setIntakeMotorPower(double percent) {
        leftIntakeMotor.set(percent);
    }

    public double getIntakeMotorAmps() {
        return leftIntakeMotor.getOutputCurrent();
    } 

    private void checkIntakeAmps() {
        if (getIntakeMotorAmps() > hardStop) {
            setIntakeMotorPower(0);
        }
    }

    @Override
    public void periodic() {
        getSetPointFromMode();
        wristControl();
        setIntakeMotorPower(shooterGoalPower);
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