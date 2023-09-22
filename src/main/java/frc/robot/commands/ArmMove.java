package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPositions;

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
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return arm.getArmPos() == ArmPositions.Warmup || arm.getArmPos() == ArmPositions.Idle;
    }



    
    
    

}