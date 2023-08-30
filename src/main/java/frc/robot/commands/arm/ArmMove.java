package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmMove extends CommandBase {

    private ArmSubsystem arm;
    private double wristGoalRad, shooterPowerGoal;

    public ArmMove (ArmSubsystem arm, double wristGoalRad, double shooterPowerGoal) {
        this.arm = arm;
        this.wristGoalRad = wristGoalRad;
        this.shooterPowerGoal = shooterPowerGoal;
    }

    @Override
    public void initialize() {
        wristGoalRad = SmartDashboard.getNumber("wrist goal rad", wristGoalRad);
        shooterPowerGoal = SmartDashboard.getNumber("shooter power goal", shooterPowerGoal);
        arm.setWristGoalRads(wristGoalRad); arm.setShooterGoalPower(shooterPowerGoal);
    }


    
    
    

}