package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

    private SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions);
    }

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions, pose);

    }

    public void updateOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry.update(rotation, modulePositions);
    }
    
    public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(gyroAngle, modulePositions, pose);
    }
    
    public Pose2d getOdometryFieldToRobot() {
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Odometry Pose", field);
        SmartDashboard.putNumber("Odometry X", field.getRobotPose().getX());
        SmartDashboard.putNumber("Odometry Y", field.getRobotPose().getY());

        return odometry.getPoseMeters();
    }
    
    
}
