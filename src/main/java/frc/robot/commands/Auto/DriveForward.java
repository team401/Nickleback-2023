package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class DriveForward extends CommandBase {
    private final DriveSubsystem drive;

    private final Timer timer = new Timer();

    public DriveForward(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        drive.setArcadeDriveControls(0, 0.7);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        drive.setArcadeDriveControls(0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 1;
    }
}
