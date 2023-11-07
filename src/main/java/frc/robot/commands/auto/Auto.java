package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import frc.robot.subsystems.drive.DriveSubsystem;



public class Auto extends SequentialCommandGroup {

    private ArmSubsystem arm;
    private DriveSubsystem drive;
    private DriverStation.Alliance alliance;

    private String pathName;


    public Auto (String pathName, ArmSubsystem arm, DriveSubsystem drive) {

        this.arm = arm;
        this.drive = drive;
        this.pathName = pathName;
        this.alliance = DriverStation.getAlliance();

        
        PathPlannerPath pathGroup = PathPlannerPath.fromPathFile(pathName);

    }


    public void periodic() {
        
    }


    public void followPath() {
    // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        PathPlannerPath pathGroup = PathPlannerPath.fromPathFile(pathName);

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("ShootCube", new InstantCommand(() -> arm.setMode(Mode.SHOOT_HIGH)));
        eventMap.put("IntakeCube", new InstantCommand(() -> arm.setMode(Mode.INTAKE)));

        FollowPathWithEvents command = new FollowPathWithEvents(
            drive(pathGroup),
            pathGroup,
            eventMap    
        );
    }

    

    
}
