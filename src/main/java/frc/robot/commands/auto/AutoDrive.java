package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoDrive extends CommandBase {

    private DriveSubsystem drive;
    private PathPlannerTrajectory path;
    private PIDController xController, yController;
    private PIDController thetaController;
    private PPHolonomicDriveController controller;
    private DriverStation.Alliance alliance;

    private Timer time = new Timer();

    public AutoDrive(DriveSubsystem drive, PathPlannerTrajectory path) {

        this.alliance = DriverStation.getAlliance();

        this.drive = drive;
        this.path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);

        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new PIDController(0, 0, 0);
        thetaController.enableContinuousInput(0, 2*Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        time.reset();
        time.start();  
    }

    @Override 
    public void execute() {

        PathPlannerTrajectory.PathPlannerState goal = ((PathPlannerTrajectory.PathPlannerState) path.sample(time.get()));
        Pose2d currentRobotPose = new Pose2d(); // get from odometry
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);

        //return to drive???
    }

    @Override
    public boolean isFinished() {
        return time.get() > path.getTotalTimeSeconds();
    }

    
}
