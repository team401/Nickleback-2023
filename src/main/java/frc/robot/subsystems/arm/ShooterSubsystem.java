package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private CANSparkMax leftMotor, rightMotor;


    public ShooterSubsystem() {
        leftMotor = new CANSparkMax(1, MotorType.kBrushless);
        rightMotor = new CANSparkMax(2, MotorType.kBrushless);

        leftMotor.follow(rightMotor, true);
    }


    public double getPositionRad() {
        return rightMotor.getEncoder().getPosition();
    }

    public void setMotorPower(double percent) {
        rightMotor.set(percent);
    }

    public double getMotorAmps() {
        return rightMotor.getOutputCurrent();
    } 
    


}
