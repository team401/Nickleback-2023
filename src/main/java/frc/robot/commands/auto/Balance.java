package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Balance extends CommandBase {

    private DriveSubsystem drive;

    public Balance(DriveSubsystem drive) {
        this.drive = drive;
    }
}
