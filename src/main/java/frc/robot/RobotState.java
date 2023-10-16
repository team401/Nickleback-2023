package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.DriveConstants;
=======
>>>>>>> 6c4d1e4 (rather meaningless balance for pos 2 sorry)

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

<<<<<<< HEAD
    private SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions);
    }

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry = new SwerveDriveOdometry(DriveConstants.kinematics, rotation, modulePositions, pose);

=======
    SwerveDriveOdometry odometry;

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        odometry = new SwerveDriveOdometry(null, rotation, modulePositions, initialPose);
>>>>>>> 6c4d1e4 (rather meaningless balance for pos 2 sorry)
    }

    public void updateOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry.update(rotation, modulePositions);
    }
<<<<<<< HEAD
    
    public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(gyroAngle, modulePositions, pose);
    }
    
    public Pose2d getOdometryFieldToRobot() {
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Odometry Pose", field);

=======

    public Pose2d getOdometryFieldToRobot() {
>>>>>>> 6c4d1e4 (rather meaningless balance for pos 2 sorry)
        return odometry.getPoseMeters();
    }
    
    
}
