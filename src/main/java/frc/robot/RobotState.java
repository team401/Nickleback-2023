package frc.robot;

<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
>>>>>>> 26a4ce0 (odometry start)
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
>>>>>>> 3009abd2207eb0f5eb096ea2a07d9e86d0f513d1
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

<<<<<<< HEAD
<<<<<<< HEAD
    public void setFieldToVehicle(){
=======
    SwerveDriveOdometry odometry;

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        odometry = new SwerveDriveOdometry(null, rotation, modulePositions, initialPose);
>>>>>>> 3009abd2207eb0f5eb096ea2a07d9e86d0f513d1

    }

    public void updateOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry.update(rotation, modulePositions);
    }

    public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(gyroAngle, modulePositions, pose);
    }
    
<<<<<<< HEAD
=======
    SwerveDriveOdometry odometry;

    public void initializeOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions, Pose2d initialPose) {
        odometry = new SwerveDriveOdometry(null, rotation, modulePositions, initialPose);

    }

    public void updateOdometry(Rotation2d rotation, SwerveModulePosition[] modulePositions) {
        odometry.update(rotation, modulePositions);
    }

    public void resetOdometry(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
        odometry.resetPosition(gyroAngle, modulePositions, pose);
    }
    
=======
>>>>>>> 3009abd2207eb0f5eb096ea2a07d9e86d0f513d1
    public Pose2d getOdometryFieldToRobot() {
        return odometry.getPoseMeters();
    }
    
    
<<<<<<< HEAD
>>>>>>> 26a4ce0 (odometry start)
=======
>>>>>>> 3009abd2207eb0f5eb096ea2a07d9e86d0f513d1
}
