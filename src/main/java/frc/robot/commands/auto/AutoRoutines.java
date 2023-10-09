package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines extends SequentialCommandGroup {

    private ArmSubsystem arm;
    private DriveSubsystem drive;

    private String pathName;

    private double driveTime;

    List<PathPlannerTrajectory> pathGroup;

    public AutoRoutines (String pathName, ArmSubsystem arm, DriveSubsystem drive, double driveTime) {

        this.arm = arm;
        this.drive = drive;
        this.pathName = pathName;
        this.driveTime = driveTime;

        PathConstraints constraints = new PathConstraints(5, 5);
        pathGroup = PathPlanner.loadPathGroup(pathName, constraints);


        addRequirements(arm);
        addRequirements(drive);

        addCommands(
            homeAutoOdometry(),
            placeCube()
        );

        if (pathName.equals("1-1")) {
            addCommands(
                drive(0),
                drive(1),
                balance()
            );
        } else if (pathName.equals("1-2")) {
            addCommands(
                drive(0),
                pickUpCube(),
                drive(1),
                drive(2),
                placeCube()
            );
        }

    }


    private Command homeAutoOdometry() {
        return new InstantCommand(() -> {
            Trajectory.State start = pathGroup.get(0).sample(0);
            //set odometry to start
        });
    }

    private Command placeCube() {
        return new ParallelRaceGroup(
            new ArmMove(arm, Mode.SHOOT_HIGH),
            new WaitCommand(1),
            new ArmMove(arm, Mode.IDLE)
        );
    }

    private Command pickUpCube() {
        return new ParallelRaceGroup(
            new ArmMove(arm, Mode.INTAKE),
            new WaitCommand(1),
            new ArmMove(arm, Mode.IDLE)
        );
    }

    private Command drive(int pathNum) {
        return new InstantCommand(() -> {
            new AutoDrive(drive, pathGroup.get(pathNum));
        });
    }

    private Command balance() {
        return new InstantCommand(() -> {
            new Balance(drive);
        });
    }

}
