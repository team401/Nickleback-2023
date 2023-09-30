package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BasicDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Auto extends SequentialCommandGroup {


    public Auto(ArmSubsystem arm, DriveSubsystem drive, double driveTime) {

        addCommands(
            new ArmMove(arm, Mode.SHOOT_HIGH).raceWith(new WaitCommand(1.0)),
            new InstantCommand(() -> drive.setArcadeDriveControls(1,0)),
            new WaitCommand(driveTime),
            new InstantCommand(() -> drive.setArcadeDriveControls(0, 0))
        );

    }


}
