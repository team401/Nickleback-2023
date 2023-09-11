package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.arm.ArmSubsystem.ArmPositions;

public class ArmMove extends CommandBase {

    private ArmSubsystem arm;
    private ArmPositions armPos;

    public ArmMove (ArmSubsystem arm, ArmPositions armPos) {
        this.arm = arm;
        this.armPos = armPos;
    }

    @Override
    public void initialize() {
        arm.setMode(armPos);
    }

   
    @Override
    public void end(boolean interrupted) {
        arm.setMode(ArmPositions.Idle);
    }


    
    
    

}