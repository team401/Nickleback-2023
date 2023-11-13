package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;



public class Auto extends SequentialCommandGroup {

    private ArmSubsystem arm;
    private DriveSubsystem drive;
    private Optional<Alliance> alliance;

    private String pathName;


    public Auto (String pathName, ArmSubsystem arm, DriveSubsystem drive) {

        this.arm = arm;
        this.drive = drive;
        this.pathName = pathName;
        this.alliance = DriverStation.getAlliance();

        
        PathPlannerPath pathGroup = PathPlannerPath.fromPathFile(pathName);

        List<Pair<String, Command>> commands = new ArrayList<Pair<String, Command>>();
        commands.add(new Pair<String, Command> ("intakeCube", new InstantCommand(() -> arm.setMode(Mode.SHOOT_HIGH))));
        commands.add(new Pair<String, Command> ("shootCube", new InstantCommand(() -> arm.setMode(Mode.INTAKE))));
    
        NamedCommands.registerCommands(commands);

        addCommands(followPath());

    }


    //

    public FollowPathWithEvents followPath() {
        PathPlannerPath pathGroup = PathPlannerPath.fromPathFile(pathName);

        Supplier<Pose2d> poseSupplier = () -> RobotState.getInstance().getOdometryFieldToRobot(); 
        Supplier<ChassisSpeeds> speedsSupplier = () -> drive.getChassisSpeeds();
        Consumer<ChassisSpeeds> outputRobotRelative = chassisSpeeds -> drive.setGoalChassisSpeeds(chassisSpeeds);


        Command followPathHolonomic = new FollowPathHolonomic(
            pathGroup, 
            poseSupplier, 
            speedsSupplier, 
            outputRobotRelative,
            new PIDConstants(0, 0, 0), 
            new PIDConstants(0, 0, 0),
            5.0,
            0.1,
            0.02,
            new ReplanningConfig(),
            drive
        );


        FollowPathWithEvents command = new FollowPathWithEvents(
            followPathHolonomic,
            pathGroup,
            poseSupplier  
        );


        return command;

    }




    
}
