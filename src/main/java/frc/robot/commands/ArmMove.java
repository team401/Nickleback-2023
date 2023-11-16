package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystemHardware;
import frc.robot.subsystems.arm.ArmSubsystemHardware.Mode;

public class ArmMove extends CommandBase {

    private ArmSubsystemHardware arm;
    private Mode mode;

    public ArmMove (ArmSubsystemHardware arm, Mode mode) {
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