package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
    private PIDController wristController = new PIDController(0, 0, 0);
    private double wristkP = 0.0;
    private double wristkI = 0.0;
    private double wristkD = 0.0;


    private static double wristGoalPosition = 0;
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

        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.getEncoder().setPosition(0);

        wristMotor.setInverted(false);

        
    }
    
    public void setMode(Modes mode) {
        currentMode = mode;
    }

    public void getSetPointFromMode (){
        SmartDashboard.putString("Mode", currentMode.toString());
        if (this.currentMode == Modes.INTAKE) {
            wristGoalPosition = wristIntake;
            shooterGoalPower = shooterIntake;
            //intakeGoalPower = intakeOn;
        } 
        else if (this.currentMode == Modes.SHOOT) {
            wristGoalPosition = wristShoot;
            shooterGoalPower = shooterShoot;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Modes.WARMUP) {
            wristGoalPosition = wristShoot;
            shooterGoalPower = shooterWarmup;
            //intakeGoalPower = intakeOff;
        }
        else {
            //IDLE MODE
            // wristGoalPosition = 0;
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
            wristGoalPosition = getWristPositionRad();
        } 
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
            setMode(Modes.IDLE);
        }
    }


    public void wristControl() {
        double start = getWristPositionRad();
        double x = wristController.calculate(start, wristGoalPosition);
        setWristMotorPower(x);
        checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPositionRad()-wristGoalPosition) < wristTolerance;
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
        wristControl();

        SmartDashboard.putNumber("wrist position radians", getWristPositionRad());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());

        wristIntake = SmartDashboard.getNumber("wrist intake", 0);
        wristWarmup = SmartDashboard.getNumber("wrist shoot rad", 0.5);

        shooterIntake = SmartDashboard.getNumber("intake power", 0.5);
        shooterShoot = SmartDashboard.getNumber("shoot power", -1);

        wristkP = SmartDashboard.getNumber("wrist kP", 0.0);
        wristkI = SmartDashboard.getNumber("wrist kI", 0.0);
        wristkD = SmartDashboard.getNumber("wrist kD", 0.0);

        SmartDashboard.putNumber("wrist intake", wristIntake);
        SmartDashboard.putNumber("wrist shoot", wristWarmup);
        SmartDashboard.putNumber("intake power", shooterIntake);
        SmartDashboard.putNumber("shoot power", shooterShoot);

        SmartDashboard.putNumber("wrist kP", wristkP);
        SmartDashboard.putNumber("wrist kI", wristkI);
        SmartDashboard.putNumber("wrist kD", wristkD);

        wristController.setPID(wristkP, wristkI, wristkD);


        // checkIntakeAmps();
    }


}