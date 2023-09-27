package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Auto extends CommandBase {
    
    private ArmSubsystem arm;
    private DriveSubsystem drive;

    private Timer timer = new Timer();
    private double timeOut = 100000;

    public Auto(ArmSubsystem arm, DriveSubsystem drive) {
        this.arm = arm;
        this.drive = drive;
    }

    public void Auto13() {
        timer.restart();

        arm.setMode(ArmPositions.ShootHigh);

        new InstantCommand(arm::shoot).schedule();
        ///schedule a warmup?

        timer.start();
        timeOut = 5;

        drive.setTankDriveControls(1, 1);

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(timeOut);
    }


}
