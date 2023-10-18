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


    private final SwerveModules[] driveModules = new SwerveModules[4]; // TODO: This will be the future format: FL, FR, BR, BL
    private final PigeonInterface pigeonInterface = new PigeonInterface();

    private boolean babyMode = false;


    /**
     * @param chassisMode
     * @param driveMode
     */
    public DriveSubsystem() {

        driveModules[0] = new SwerveModules(DriveConstants.frontLeftDriveID, DriveConstants.frontLeftRotationMotorID,
        DriveConstants.frontLeftRotationEncoderID, DriveConstants.frontLeftAngleOffset, false, false);
        driveModules[1] = new SwerveModules(DriveConstants.frontRightDriveID, DriveConstants.frontRightRotationMotorID,
        DriveConstants.frontRightRotationEncoderID, DriveConstants.frontRightAngleOffset, true, false);
        driveModules[2] = new SwerveModules(DriveConstants.backRightDriveID, DriveConstants.backRightRotationMotorID,
        DriveConstants.backRightRotationEncoderID, DriveConstants.backRightAngleOffset, false, false);
        driveModules[3] = new SwerveModules(DriveConstants.backLeftDriveID, DriveConstants.backLeftRotationMotorID,
        DriveConstants.backLeftRotationEncoderID, DriveConstants.backLeftAngleOffset, false, false);

        for (int i = 0; i < 4; i++) {
            // TODO: Maybe add specific PID values for each module
            driveModules[i].zeroEncoders();

            driveModules[i].setDrivePD(DriveConstants.driveKp, DriveConstants.driveKd);
            
            driveModules[i].initModulePosition();
        }

        //TODO: add swerve drive odometry
        setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        RobotState.getInstance().initializeOdometry(getRotation(), getSwerveModulePositions());
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

        RobotState.getInstance().updateOdometry(getRotation(), getSwerveModulePositions());

        SmartDashboard.putNumber("Drive/velocity magnitude", getChassisSpeeds().vxMetersPerSecond);

        SmartDashboard.putNumber("Encoder Position 1", new Rotation2d(driveModules[0].getRotationPosition()).getRotations());
        SmartDashboard.putNumber("Encoder Position 2", new Rotation2d(driveModules[1].getRotationPosition()).getRotations());
        SmartDashboard.putNumber("Encoder Position 3", new Rotation2d(driveModules[2].getRotationPosition()).getRotations());
        SmartDashboard.putNumber("Encoder Position 4", new Rotation2d(driveModules[3].getRotationPosition()).getRotations());
    }


    public void setGoalModuleStates(SwerveModuleState[] states) {
        double[] array = new double[8];
        for (int i = 0; i < 4; i++) {
            driveModules[i].setGoalModuleState(states[i]);
            array[i*2] = driveModules[i].getRotationPosition();
            array[i*2+1] = driveModules[i].getDrivePosition();
        }
        SmartDashboard.putNumberArray("Drive/moduleStates/State", array);
    }

    /**
     * Set the desired velocities of the robot drive (xVelocity, yVelocity, omegaVelocity)
     * @param speeds the desired speed of the robot
     */
    public void setGoalChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * (babyMode ? 0.2 : 1), speeds.vyMetersPerSecond * (babyMode ? 0.2 : 1), speeds.omegaRadiansPerSecond * (babyMode ? 0.1 : 1));
        SwerveModuleState[] goalModuleStates = DriveConstants.kinematics.toSwerveModuleStates(speeds);

        // Convert back to spiral order.
        SwerveModuleState tmp = goalModuleStates[2];
        goalModuleStates[2] = goalModuleStates[3];
        goalModuleStates[3] = tmp;

        SwerveDriveKinematics.desaturateWheelSpeeds(goalModuleStates, DriveConstants.maxDriveSpeed);
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            goalModuleStates = new SwerveModuleState[] {
                new SwerveModuleState(0, driveModules[0].getModulePosition().angle), 
                new SwerveModuleState(0, driveModules[1].getModulePosition().angle),
                new SwerveModuleState(0, driveModules[2].getModulePosition().angle),
                new SwerveModuleState(0, driveModules[3].getModulePosition().angle)
            };
        }
        double[] moduleGoalStates = {
            driveModules[0].getRotationVelocity(),
            driveModules[0].getDriveVelocityMPerS(),
            driveModules[1].getRotationVelocity(),
            driveModules[1].getDriveVelocityMPerS(),
            driveModules[2].getRotationVelocity(),
            driveModules[2].getDriveVelocityMPerS(),
            driveModules[3].getRotationVelocity(),
            driveModules[3].getDriveVelocityMPerS()
        };
        SmartDashboard.putNumberArray("Drive/moduleStates/Goal", moduleGoalStates);
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

    public void setFieldToVehicle(Pose2d fieldToVehicle) {
        RobotState.getInstance().resetOdometry(getRotation(), getSwerveModulePositions(), fieldToVehicle);
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

    public SwerveModulePosition[] getSwerveModulePositions() {

        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = driveModules[i].getModulePosition();
        }
        return swerveModulePositions;
        
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


