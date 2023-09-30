package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClawMode;
import frc.robot.subsystems.ClawSubsystem;

public class ClawMove extends CommandBase {
    private ClawSubsystem clawSub;
    private ClawMode clawMode;

    public ClawMove (ClawSubsystem claw, ClawMode mode){
        clawSub = claw;
        clawMode = mode;
    }

    @Override
    public void initialize(){
        clawSub.setClawMode(clawMode);
    }

    @Override
    public void end (boolean interrupted){
        clawSub.setClawMode(ClawMode.Idle);
    }
}
