package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
=======
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)
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
<<<<<<< HEAD
import frc.robot.RobotState;
=======
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines extends SequentialCommandGroup {

<<<<<<< HEAD
    private ArmSubsystem arm;
    private DriveSubsystem drive;

    private String pathName;

    List<PathPlannerTrajectory> pathGroup;

    //private double shootLine = 0; find position for shooting(?)

    public AutoRoutines (String pathName, ArmSubsystem arm, DriveSubsystem drive) {

        this.arm = arm;
        this.drive = drive;
        this.pathName = pathName;
        PathConstraints constraints = new PathConstraints(5, 5); //TODO: get max values
        pathGroup = PathPlanner.loadPathGroup(pathName, constraints);


        addCommands(
            homeAutoOdometry(),
            placeCube()
        );

        if (pathName.indexOf("1-1_Cube") != -1 && pathName.indexOf("3-1_Cube") != -1) {
            addCommands(
                drive(0),
                drive(1),
                balance()
            );
        } else if (pathName.indexOf("1-2_Cube") != -1 && pathName.indexOf("3-2_Cube") != -1) {
            addCommands(
                new ParallelRaceGroup(
                    drive(0),
                    new WaitCommand(2).andThen(pickUpCube())
                ),
                drive(1),
                drive(2),
                placeCube()
            );
        } else if (pathName.indexOf("2-Balance") != 0) {
            addCommands(
                new ParallelRaceGroup(
                    drive(0)
                )
            );
        }

        if (pathName.indexOf("Balance") != -1) {
            addCommands(
                balance()
            );
        }

    }


    private Command homeAutoOdometry() {
        return new InstantCommand(() -> {
            Trajectory.State start = pathGroup.get(0).sample(0);
            drive.setFieldToVehicle(new Pose2d(start.poseMeters.getTranslation(), Rotation2d.fromRadians(start.curvatureRadPerMeter)));
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
=======

    


>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)


}