package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;

public class ArmMove extends CommandBase {

    private ArmSubsystem arm;
    private Mode mode;

    public ArmMove (ArmSubsystem arm, Mode mode) {
        this.arm = arm;
        this.mode = mode;
    }

    @Override
    public void initialize() {
        arm.setMode(this.mode);
    }

    @Override
    public void end (boolean interupted){
        arm.setMode(Mode.STOW);
    }
}