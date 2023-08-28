package frc.robot.commands.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.WristSubsystem;

public class WristMove extends CommandBase {

    private WristSubsystem wrist;
    private double goalRad;
    private PIDController controller;

    public WristMove (WristSubsystem wrist, double goalRad) {
        this.wrist = wrist;
        this.goalRad = goalRad;
        controller = new PIDController(0, 0, 0);
    }

    @Override
    public void execute() {
        // add TrapezoidProfile + feedforward later?
        double start = wrist.getPositionRad();
        double x = controller.calculate(goalRad, start);
        wrist.setMotorPower(x);
    }

}
