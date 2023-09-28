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
    private final double softStop = 20.0;
    private final double hardStop = 25.0;

    private CANSparkMax leftIntakeMotor;
    private CANSparkMax rightIntakeMotor;

    private double shooterGoalPower;
    //private double intakeGoalPower;

    // SmartDashboard setpoint values
    private double wristIntakePosition;
    private double wristShootPosition;

    // SmartDashboard 
    private double shooterIntakePower = 0.5;
    private double shooterShootPower = -1;
    private double shooterWarmup = 0;

    //private double intakeOn, intakeOff = 0;


    public static enum Mode {
        IDLE,
        INTAKE,
        WARMUP,
        SHOOT;
    }

    private Mode currentMode = Mode.IDLE;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
    
        leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorID, MotorType.kBrushless);
        rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorID, MotorType.kBrushless);

        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.getEncoder().setPosition(0);

        wristMotor.setInverted(false);
    }
    
    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void getSetPointFromMode (){
        SmartDashboard.putString("Mode", currentMode.toString());

        if (this.currentMode == Mode.INTAKE) {
            wristGoalPosition = wristIntakePosition;
            shooterGoalPower = shooterIntakePower;
            //intakeGoalPower = intakeOn;
        } 
        else if (this.currentMode == Mode.SHOOT) {
            wristGoalPosition = wristShootPosition;
            shooterGoalPower = shooterShootPower;
            //intakeGoalPower = intakeOff;
        } 
        else if (this.currentMode == Mode.WARMUP) {
            wristGoalPosition = wristShootPosition;
            shooterGoalPower = shooterWarmup;
            //intakeGoalPower = intakeOff;
        }
        else {
            //IDLE MODE
            // wristGoalPosition = 0;
        }
    }
  
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    public double getWristPosition() {
        return wristMotor.getEncoder().getPosition();
    }

    private void setWristMotorPower(double percent) {
        wristMotor.set(percent);
    }

    public double getWristMotorAmps() {
        return wristMotor.getOutputCurrent();
    }

    private void checkWristAmps() {
        if (getWristMotorAmps() > softStop) {
            wristGoalPosition = getWristPosition();
        } 
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
            setMode(Mode.IDLE);
        }
    }


    public void wristControl() {
        double start = getWristPosition();
        double x = wristController.calculate(start, wristGoalPosition);
        setWristMotorPower(x);
        checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPosition()-wristGoalPosition) < wristTolerance;
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
        setIntakeMotorPower(shooterGoalPower);
        wristControl();

        SmartDashboard.putNumber("wrist position", getWristPosition());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());

        wristIntakePosition = SmartDashboard.getNumber("wrist intake position", 0);
        wristShootPosition = SmartDashboard.getNumber("wrist shoot position", 0.5);

        shooterIntakePower = SmartDashboard.getNumber("intake power", 0.5);
        shooterShootPower = SmartDashboard.getNumber("shoot power", -1);

        wristkP = SmartDashboard.getNumber("wrist kP", 0.0);
        wristkI = SmartDashboard.getNumber("wrist kI", 0.0);
        wristkD = SmartDashboard.getNumber("wrist kD", 0.0);

        wristController.setPID(wristkP, wristkI, wristkD);

        SmartDashboard.putNumber("wrist intake position", wristIntakePosition);
        SmartDashboard.putNumber("wrist shoot position", wristShootPosition);

        SmartDashboard.putNumber("intake power", shooterIntakePower);
        SmartDashboard.putNumber("shoot power", shooterShootPower);

        SmartDashboard.putNumber("wrist kP", wristkP);
        SmartDashboard.putNumber("wrist kI", wristkI);
        SmartDashboard.putNumber("wrist kD", wristkD);
        // checkIntakeAmps();
    }


}