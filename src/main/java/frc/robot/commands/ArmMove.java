package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.Mode;

public class ArmMove extends Command {

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