package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClawMode;


public class ClawSubsystem extends SubsystemBase{
    private CANSparkMax clawMotor;


    private ClawMode currentMode = ClawMode.Idle;

    public ClawSubsystem (){
        clawMotor = new CANSparkMax(14, MotorType.kBrushless);
        clawMotor.setIdleMode(IdleMode.kBrake);
        clawMotor.setSmartCurrentLimit(10);
    }

    public void setClawMode(ClawMode mode){
        this.currentMode = mode;
    }

    @Override
    public void periodic(){
        switch(currentMode){
            case Opening:
                clawMotor.set(Constants.clawPower);
                break;
            case Closing:
                clawMotor.set(-Constants.clawPower);
                break;
            case Idle:
            default:
                clawMotor.set(0.0);
                break;
        }
    
    }
}
