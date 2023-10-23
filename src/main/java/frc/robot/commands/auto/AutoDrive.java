package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoDrive extends CommandBase {

    private DriveSubsystem drive;
    private PathPlannerTrajectory path;
    
    private DriverStation.Alliance alliance;

    public AutoDrive(DriveSubsystem drive, PathPlannerTrajectory path) {

        this.alliance = DriverStation.getAlliance();

        this.drive = drive;
        this.path = PathPlannerTrajectory.transformTrajectoryForAlliance(path, alliance);

        addRequirements(drive);

    }

    @Override 
    public void initialize() {
        drive.driveByPath(path);
    }

    @Override
    public boolean isFinished() {
        return drive.pathIsFinished();
    }

    @Override
    public void end(boolean interrupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

    
}
