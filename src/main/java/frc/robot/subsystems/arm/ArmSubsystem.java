package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Intake;



public class ArmSubsystem extends SubsystemBase{
    
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



    public static enum Mode {
        IDLE,
        INTAKE,
        STOW,
        SHOOT_HIGH,
        SHOOT_MID,
        SHOOT_LOW,
        SPIT;
    }

    private Mode currentMode = Mode.IDLE;
    private double wristGoalPosition = 0.0;

    public ArmSubsystem() {
        wristMotor = new CANSparkMax(Constants.ArmConstants.wristMotorID, MotorType.kBrushless);
    
        intake = new Intake();

        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.getEncoder().setPosition(0);

        wristMotor.setInverted(false);

        // I know, this looks bad
        wristMotor.setSmartCurrentLimit(30);
    }
    
    public void setMode(Mode mode) {
        currentMode = mode;
    }

    public void getSetpointFromMode (){
        SmartDashboard.putString("Mode", currentMode.toString());

        if (this.currentMode == Mode.INTAKE) {
            wristGoalPosition = ArmConstants.intakePosition;
            intake.intake();
        } 
        else if (this.currentMode == Mode.SHOOT_HIGH) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            intake.shootHigh();
        } 
        else if (this.currentMode == Mode.SHOOT_MID) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            intake.shootMid();
        } 
        else if (this.currentMode == Mode.SHOOT_LOW) {
            wristGoalPosition = ArmConstants.upperShootPosition;
            intake.shootLow();
        } 
        else if (this.currentMode == Mode.STOW) {
            wristGoalPosition = ArmConstants.stowShootPosition;
            intake.off();
        }
        else if (this.currentMode == Mode.SPIT){
            wristGoalPosition = ArmConstants.intakePosition;
            if (wristFinished()){
                intake.spit();
            }
            else{
                intake.off();
            }
        }
        else {
            //IDLE MODE
            wristGoalPosition = ArmConstants.stowShootPosition;
        }
    }

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
        if (getWristMotorAmps() > hardStop) {
            setWristMotorPower(0);
            setMode(Mode.IDLE);
        }
    }


    public void wristControl() {
        double output = wristController.calculate(getWristPosition(), wristGoalPosition);
        setWristMotorPower(output);
        // checkWristAmps();
    }

    public boolean wristFinished() {
        return Math.abs(getWristPosition()-wristGoalPosition) < wristTolerance;
    }

    @Override
    public void periodic() {
        getSetpointFromMode();
        wristControl();

        SmartDashboard.putNumber("wrist position", getWristPosition());
        SmartDashboard.putNumber("wrist amps", getWristMotorAmps());
        SmartDashboard.putBoolean("wrist finished", wristFinished());

        wristIntakePosition = SmartDashboard.getNumber("wrist intake position", 0);
        wristShootPosition = SmartDashboard.getNumber("wrist shoot position", 0.5);

        shooterIntakePower = SmartDashboard.getNumber("intake power", 0.5);
        shooterShootPower = SmartDashboard.getNumber("shoot power", -1);

        wristkP = SmartDashboard.getNumber("wrist kP", 0.03);
        wristkI = SmartDashboard.getNumber("wrist kI", 0.0);
        wristkD = SmartDashboard.getNumber("wrist kD", 0.0);

        wristController.setPID(wristkP, wristkI, wristkD);

        intake.run();

        SmartDashboard.putNumber("wrist intake position", wristIntakePosition);
        SmartDashboard.putNumber("wrist shoot position", wristShootPosition);

        SmartDashboard.putNumber("intake power", shooterIntakePower);
        SmartDashboard.putNumber("shoot power", shooterShootPower);

        SmartDashboard.putNumber("wrist kP", wristkP);
        SmartDashboard.putNumber("wrist kI", wristkI);
        SmartDashboard.putNumber("wrist kD", wristkD);
    }


}