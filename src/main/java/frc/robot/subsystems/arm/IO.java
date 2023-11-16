package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Simulator extends Subsystem { 
    
    public double getWristPosition();
    public double setWristMotorPower();
    public double getWristMotorAmps();
    
}
