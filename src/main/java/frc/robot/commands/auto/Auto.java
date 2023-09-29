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
import frc.robot.subsystems.ArmSubsystem.ArmPositions;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Auto extends SequentialCommandGroup {


    public Auto(ArmSubsystem arm, DriveSubsystem drive, double driveTime) {

        addCommands(
            new InstantCommand(() -> arm.setMode(ArmPositions.ShootHigh)),
            new WaitCommand(0.5)
                .andThen(new InstantCommand(arm::shoot)),
            new WaitCommand(0.5)
                .andThen(new InstantCommand(arm::stopIntake))
                .andThen(new InstantCommand(() -> arm.setMode(ArmPositions.Idle))),
            new ParallelRaceGroup(
                new WaitCommand(driveTime),
                new InstantCommand(() -> drive.setTankDriveControls(1,1))
            ),
            new InstantCommand(() -> drive.setTankDriveControls(0, 0))
        );

    }


}
