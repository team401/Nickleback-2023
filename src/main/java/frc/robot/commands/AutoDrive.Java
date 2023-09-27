package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoDrive extends CommandBase{
    Timer timer = new Timer();
    private final DriveSubsystem drivesub;

    public AutoDrive(DriveSubsystem d){
        drivesub = d;
        addRequirements(drivesub);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        drivesub.setArcadeDriveControls(0, 0.75);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted){
        drivesub.setArcadeDriveControls(0,0);
    }

    @Override
    public boolean isFinished(){
        return timer.get() > 1;
    }
}