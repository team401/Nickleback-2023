package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;


public class WristSubsystem {
    
    private CANSparkMax wristMotor;

    public WristSubsystem() {
        wristMotor = new CANSparkMax(0, MotorType.kBrushless);

    }

    public double getPositionRad() {
        return wristMotor.getEncoder().getPosition();
    }

    public void setMotorPower(double percent) {
        wristMotor.set(percent);
    }

}
