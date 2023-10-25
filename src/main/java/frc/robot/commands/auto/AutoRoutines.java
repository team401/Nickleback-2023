package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import frc.robot.commands.ArmMove;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoRoutines extends SequentialCommandGroup {

    private ArmSubsystem arm;
    private DriveSubsystem drive;
    private DriverStation.Alliance alliance;

    private String pathName;

    private double shootLine = 0; //TODO: find position for shooting

    public AutoRoutines (String pathName, ArmSubsystem arm, DriveSubsystem drive) {

        this.arm = arm;
        this.drive = drive;
        this.pathName = pathName;
        this.alliance = DriverStation.getAlliance();
        PathConstraints constraints = new PathConstraints(5, 5); //TODO: get max values
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(pathName, constraints);


        addCommands(
            homeAutoOdometry(pathGroup),
            placeCube()
        );

       if (pathName.indexOf("1-2_Cube") != -1) {
            addCommands(
                new ParallelRaceGroup(
                    drive(pathGroup.get(0)),
                    new WaitCommand(2).andThen(pickUpCube())
                ),
                drive(pathGroup.get(1)),
                placeCube()
            );
        } else if (pathName.indexOf("2-0_Cube") != 0) {
            //does nothing
        }

        if (pathName.indexOf("Balance") != -1) {
            addCommands(
                balance()
            );
        }

    }

    private Command homeAutoOdometry(List<PathPlannerTrajectory> pathGroup) {
        return new InstantCommand(() -> {
            Trajectory.State start = pathGroup.get(0).sample(0);
            //TODO: set odometry to start
        });
    }
    private Command placeCube() {
        return new SequentialCommandGroup(
            new ArmMove(arm, Mode.SHOOT_HIGH).raceWith(new WaitCommand(1)),
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

    private Command drive(PathPlannerTrajectory pathN) {
        
        PathPlannerTrajectory path = PathPlannerTrajectory.transformTrajectoryForAlliance(pathN, alliance);
        
        return new InstantCommand(() -> {
            drive.driveByPath(path);
        });
    }

    private Command balance() {
        return new InstantCommand(() -> {
            drive.driveByBalance();
        });
    }


}
