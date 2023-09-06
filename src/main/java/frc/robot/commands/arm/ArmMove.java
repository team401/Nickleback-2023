package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmMove extends CommandBase {

    private ArmSubsystem arm;
    private double wristGoalRad, shooterPowerGoal;
    private boolean intakeMotorStatus;

    public ArmMove (ArmSubsystem arm, double wristGoalRad, double shooterPowerGoal, boolean intakeMotorStatus) {
        this.arm = arm;
        this.wristGoalRad = wristGoalRad;
        this.shooterPowerGoal = shooterPowerGoal;
        this.intakeMotorStatus = intakeMotorStatus;
    }

    @Override
    public void initialize() {
        wristGoalRad = SmartDashboard.getNumber("wrist goal rad", wristGoalRad);
        shooterPowerGoal = SmartDashboard.getNumber("shooter power goal", shooterPowerGoal);
        intakeMotorStatus = SmartDashboard.getBoolean("intake motor activated", intakeMotorStatus);
        arm.setWristGoalRads(wristGoalRad); arm.setShooterGoalPower(shooterPowerGoal); 
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }


    
    
    

}