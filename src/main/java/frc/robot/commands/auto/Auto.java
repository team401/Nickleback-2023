package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.arm.ArmSubsystemHardware;
import frc.robot.subsystems.arm.ArmSubsystemHardware.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Auto extends SequentialCommandGroup {


    public Auto(ArmSubsystemHardware arm, DriveSubsystem drive, double driveTime) {
        addRequirements(arm);
        addRequirements(drive);

        addCommands(
            new ArmMove(arm, Mode.SHOOT_HIGH).raceWith(new WaitCommand(1.0)),
            new ParallelRaceGroup (
                new ArmMove(arm, Mode.INTAKE),
                new RunCommand(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0.5, 0, 0))),
                new WaitCommand(driveTime)
            ),
            new InstantCommand(() -> SmartDashboard.putBoolean("drive finished", true)),
            new InstantCommand(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0))),
            new ArmMove(arm, Mode.STOW)
        );  
    }
}
