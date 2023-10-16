package frc.robot.subsystems.drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;

public class DriveSubsystem extends SubsystemBase{


    private final SwerveModules[] driveModules = new SwerveModules[4];
    private final PigeonInterface pigeonInterface = new PigeonInterface(); 

    private boolean babyMode = false;


    /**
     * @param chassisMode
     * @param driveMode
     */
    public DriveSubsystem() {

        driveModules[0] = new SwerveModules(DriveConstants.frontLeftID, DriveConstants.frontLeftRotationMotorID,
        DriveConstants.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset, false, false);
        driveModules[1] = new SwerveModules(DriveConstants.frontRightID, DriveConstants.frontRightRotationMotorID,
        DriveConstants.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset, true, false);
        driveModules[2] = new SwerveModules(DriveConstants.backLeftID, DriveConstants.backLeftRotationMotorID,
        DriveConstants.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset, false, false);
        driveModules[3] = new SwerveModules(DriveConstants.backRightID, DriveConstants.backRightRotationMotorID,
        DriveConstants.backRightRotationEncoderID, DriveConstants.backRightAngleOffset, true, false);

        for (int i = 0; i < 4; i++) {
            driveModules[i].zeroEncoders();

            driveModules[i].setDrivePD(DriveConstants.driveKps, DriveConstants.driveKds);
            
            driveModules[i].initModulePosition();
        }

        //TODO: add swerve drive odometry
        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        RobotState.getInstance();//.initializePoseEstimator(getRotation(), modulePositions);
    }

    
    @Override
    public void periodic(){
        for (int i = 0; i < 4; i++) {
            //Get encoder value
            Rotation2d moduleRotation = new Rotation2d(driveModules[i].getRotationPosition());

            //Optimize each module state
            SwerveModuleState optimizedState = SwerveModuleState.optimize(driveModules[i].getModuleState(), moduleRotation);
            driveModules[i].moduleControl(optimizedState);

        }

        //estimation of position
        for (int i = 0; i < 4; i++) {
            driveModules[i].getModulePosition().distanceMeters = driveModules[i].getDrivePosition() * DriveConstants.wheelRadiusM;
            driveModules[i].getModulePosition().angle = new Rotation2d(driveModules[i].getRotationPosition());

            SmartDashboard.putNumber("Drive/modules/"+i+"/angle", driveModules[i].getModulePosition().angle.getRadians());
            SmartDashboard.putNumber("Drive/modules/"+i+"/drive stator current", driveModules[i].getDriveStatorCurrent());
            SmartDashboard.putNumber("Drive/modules/"+i+"/rotation stator current", driveModules[i].getRotationStatorCurrent());
        }

        //TODO: add swerve odometry
        RobotState.getInstance().recordDriveObservations(/*getRotation(), modulePositions*/);

        SmartDashboard.putNumber("Drive/velocity magnitude", getChassisSpeeds().vxMetersPerSecond);
    }


    public void setGoalModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setGoalModuleState(states[i]);
        }
    }

    /**
     * Set the desired velocities of the robot drive (xVelocity, yVelocity, omegaVelocity)
     * @param speeds the desired speed of the robot
     */
    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * (babyMode ? 0.2 : 1), speeds.vyMetersPerSecond * (babyMode ? 0.2 : 1), speeds.omegaRadiansPerSecond * (babyMode ? 0.1 : 1));
        SwerveModuleState[] goalModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(goalModuleStates, DriveConstants.maxDriveSpeed);
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            goalModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, driveModules[0].getModulePosition().angle), 
                new SwerveModuleState(0, driveModules[1].getModulePosition().angle),
                new SwerveModuleState(0, driveModules[2].getModulePosition().angle),
                new SwerveModuleState(0, driveModules[3].getModulePosition().angle)
            };
        }
        setGoalModuleStates(goalModuleStates);
    }

    public Rotation2d getRotation() {
        return new Rotation2d(MathUtil.angleModulus(pigeonInterface.getHeading()));
    }

        public void resetHeading() {
        pigeonInterface.resetHeading();
    }

    public void setHeading(double heading) {
        pigeonInterface.setHeading(heading);
    }

    public double getRoll() {
        return pigeonInterface.getRoll();
    }

    public void setBabyMode(boolean baby) {
        babyMode = baby;
    }

    //TODO: odometry
    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        RobotState.getInstance().setFieldToVehicle(/*getRotation(), modulePositions, fieldToVehicle*/);
    }

    public void setVolts(double v) {
        driveModules[0].setDriveVoltage(v);
        driveModules[1].setDriveVoltage(v);
        driveModules[2].setDriveVoltage(v);
        driveModules[3].setDriveVoltage(v);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = driveModules[i].getModuleState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    public void setBrakeMode(boolean braked) {
        for (int i = 0; i < 4; i++) {
            driveModules[i].setBrake(braked);
        }
    }
} 


