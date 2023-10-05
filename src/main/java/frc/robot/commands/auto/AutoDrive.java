package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
<<<<<<< HEAD
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoDrive extends CommandBase {

    private DriveSubsystem drive;
    private PathPlannerTrajectory path;
<<<<<<< HEAD

    private PIDController xController, yController;
    private PIDController thetaController;

    //TODO: get pid values
    private double kPX = 0, kIX = 0, kDX = 0, kPY = 0, kIY = 0, kDY = 0, kPTheta = 0, kITheta = 0, kDTheta = 0;

    private PPHolonomicDriveController controller;
    private DriverStation.Alliance alliance;
=======
    private PIDController xController, yController;
    private ProfiledPIDController thetaController;
    private HolonomicDriveController controller;
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)

    private Timer time = new Timer();

    public AutoDrive(DriveSubsystem drive, PathPlannerTrajectory path) {
<<<<<<< HEAD

        this.alliance = DriverStation.getAlliance();

        this.drive = drive;
        this.path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);

        xController = new PIDController(kPX, kIX, kPX);
        yController = new PIDController(kPY, kIY, kDY);
        thetaController = new PIDController(kPTheta, kITheta, kDTheta);
        thetaController.enableContinuousInput(0, 2*Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);
=======
        this.drive = drive;
        this.path = path;

        xController = new PIDController(0, 0, 0);
        yController = new PIDController(0, 0, 0);
        thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.0,0.0));
        thetaController.enableContinuousInput(0, 2*Math.PI);

        controller = new HolonomicDriveController(xController, yController, thetaController);
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)

        addRequirements(drive);

    }

    @Override
    public void initialize() {
        time.reset();
        time.start();  
<<<<<<< HEAD

        SmartDashboard.putNumber("kP X", kPX);
        SmartDashboard.putNumber("kI X", kIX);
        SmartDashboard.putNumber("kD X", kDX);

        SmartDashboard.putNumber("kP Y", kPY);
        SmartDashboard.putNumber("kI Y", kIY);
        SmartDashboard.putNumber("kD Y", kDY);

        SmartDashboard.putNumber("kP Theta", kPTheta);
        SmartDashboard.putNumber("kI Theta", kITheta);
        SmartDashboard.putNumber("kD Theta", kDTheta);
=======
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)
    }

    @Override 
    public void execute() {

<<<<<<< HEAD
        PathPlannerTrajectory.PathPlannerState goal = ((PathPlannerTrajectory.PathPlannerState) path.sample(time.get()));
        Pose2d currentRobotPose = RobotState.getInstance().getOdometryFieldToRobot();
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);

        SmartDashboard.getNumber("kP X", kPX);
        SmartDashboard.getNumber("kI X", kIX);
        SmartDashboard.getNumber("kD X", kDX);

        SmartDashboard.getNumber("kP Y", kPY);
        SmartDashboard.getNumber("kI Y", kIY);
        SmartDashboard.getNumber("kD Y", kDY);

        SmartDashboard.getNumber("kP Theta", kPTheta);
        SmartDashboard.getNumber("kI Theta", kITheta);
        SmartDashboard.getNumber("kD Theta", kDTheta);

        drive.setGoalChassisSpeeds(adjustedSpeeds);
    }

    @Override
    public boolean isFinished() {
        return time.get() > path.getTotalTimeSeconds();
    }

=======
        Trajectory.State goal = path.sample(time.get());
        Pose2d currentRobotPose = new Pose2d(); // get from odometry
        
        ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal, goal.poseMeters.getRotation());

        //return to drive???
    }
>>>>>>> 6adaa83 (missing a lot of stuff, requires odometry + drive)
    
}