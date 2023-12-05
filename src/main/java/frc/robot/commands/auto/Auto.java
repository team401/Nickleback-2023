package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.management.InstanceNotFoundException;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
        

        //TODO: will events wait until the command is finished to continue or will the robot continue running while the command runs?
        
        List<Pair<String, Command>> commands = new ArrayList<Pair<String, Command>>();
        /*commands.add(new Pair<String, Command> ("shootCube", 
            new SequentialCommandGroup(
                new RunCommand(() -> arm.setMode(Mode.SHOOT_HIGH)),
                new WaitCommand(0.1),
                new RunCommand(() -> arm.setMode(Mode.STOW))
            )));

        commands.add(new Pair<String, Command> ("intakeCube", 
            new SequentialCommandGroup(
                new RunCommand(() -> arm.setMode(Mode.INTAKE)),
                new WaitCommand(1),
                new RunCommand(() -> arm.setMode(Mode.STOW))
            )));

        */

        commands.add(new Pair<String, Command> ("shootCube", 
            new InstantCommand(() -> SmartDashboard.putString("armState", "shootCube"))));

            commands.add(new Pair<String, Command> ("intakeCube", 
            new InstantCommand(() -> SmartDashboard.putString("armState", "intakeCube"))));
        
    
        NamedCommands.registerCommands(commands);

        //addCommands(new InstantCommand(() -> drive.setGoalChassisSpeeds(new ChassisSpeeds(1, 1, 0))));

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
