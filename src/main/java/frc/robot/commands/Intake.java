package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class Intake extends CommandBase {

    private ArmSubsystem arm;

    public Intake(){

    }

    @Override
    public void initialize() {
        arm.setIntakeMotorPower(0.0);
        arm.setShooterMotorPower(0.0);
    }

    public void shoot(){
        arm.setIntakeMotorPower(0.5);
        arm.setShooterMotorPower(0.5);
    }

    public void spit(){
        arm.setIntakeMotorPower(-0.5);
        arm.setShooterMotorPower(-0.5);
    }

    public void stop(){
        arm.setIntakeMotorPower(0.0);
        arm.setShooterMotorPower(0.0);
    }

}