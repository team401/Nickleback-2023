package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Rotation2d;
=======
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
>>>>>>> 26a4ce0 (odometry start)
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class RobotState {

    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        return instance;
    }

<<<<<<< HEAD
    public void setFieldToVehicle(){

    }

    public void recordDriveObservations() {

    }
    
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
    
    public Pose2d getOdometryFieldToRobot() {
        return odometry.getPoseMeters();
    }
    
    
>>>>>>> 26a4ce0 (odometry start)
}
