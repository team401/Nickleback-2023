package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmSubsystemIO extends Subsystem {

    public static enum Mode {
        IDLE,
        INTAKE,
        STOW,
        SHOOT_HIGH,
        SHOOT_MID,
        SHOOT_LOW,
        SPIT;
    }

    public double getWristPosition();

    public double getWristMotorAmps();

    public boolean wristFinished();



    
    
}
