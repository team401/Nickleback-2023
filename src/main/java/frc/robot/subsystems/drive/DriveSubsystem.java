package frc.robot.subsystems.drive;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;

public class DriveSubsystem extends SubsystemBase{


    private final SwerveModules[] driveModules = new SwerveModules[4]; // TODO: This will be the future format: FL, FR, BR, BL
    private final PigeonInterface pigeonInterface = new PigeonInterface();

    private boolean babyMode = false;

    //TODO: get pid values
    private double kPX = 0, kIX = 0, kDX = 0, kPY = 0, kIY = 0, kDY = 0, kPTheta = 0, kITheta = 0, kDTheta = 0;

    private PIDController xController, yController;
    private PIDController thetaController;

    private PPHolonomicDriveController controller;

    private Timer pathTime = new Timer();

    private PathPlannerTrajectory path;

    private static enum Control {
        PLAYER,
        PATH,
        BALANCE
    }

    private Control mode;

    private PigeonInterface pigeon = new PigeonInterface();

    private PIDController driveController, balanceController;

    private double targetStationX = 0; 
    //TODO: get location of targetStationX

    //TODO: get pid values
    private double kPDrive = 0, kIDrive = 0, kDDrive = 0, kPBalance = 0, kIBalance = 0, kDBalance = 0;

    private Timer searchTimer = new Timer();
    private Timer balanceTimer = new Timer();
    private Timer levelTime = new Timer();

    private boolean balanceFound;

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

        SmartDashboard.putNumber("kP X", kPX);
        SmartDashboard.putNumber("kI X", kIX);
        SmartDashboard.putNumber("kD X", kDX);

        SmartDashboard.putNumber("kP Y", kPY);
        SmartDashboard.putNumber("kI Y", kIY);
        SmartDashboard.putNumber("kD Y", kDY);

        SmartDashboard.putNumber("kP Theta", kPTheta);
        SmartDashboard.putNumber("kI Theta", kITheta);
        SmartDashboard.putNumber("kD Theta", kDTheta);

        SmartDashboard.getNumber("kP X", kPX);
        SmartDashboard.getNumber("kI X", kIX);
        SmartDashboard.getNumber("kD X", kDX);

        SmartDashboard.getNumber("kP Y", kPY);
        SmartDashboard.getNumber("kI Y", kIY);
        SmartDashboard.getNumber("kD Y", kDY);

        SmartDashboard.getNumber("kP Theta", kPTheta);
        SmartDashboard.getNumber("kI Theta", kITheta);
        SmartDashboard.getNumber("kD Theta", kDTheta);

        SmartDashboard.putNumber("kP Drive", kPDrive);
        SmartDashboard.putNumber("kI Drive", kIDrive);
        SmartDashboard.putNumber("kD Drive", kDDrive);

        SmartDashboard.putNumber("kP Balance", kPBalance);
        SmartDashboard.putNumber("kI Balance", kIBalance);
        SmartDashboard.putNumber("kD Balance", kDBalance);
            
        SmartDashboard.getNumber("kP Drive", kPDrive);
        SmartDashboard.getNumber("kI Drive", kIDrive);
        SmartDashboard.getNumber("kD Drive", kDDrive);

        SmartDashboard.getNumber("kP Balance", kPBalance);
        SmartDashboard.getNumber("kI Balance", kIBalance);
        SmartDashboard.getNumber("kD Balance", kDBalance);


        xController = new PIDController(kPX, kIX, kPX);
        yController = new PIDController(kPY, kIY, kDY);
        thetaController = new PIDController(kPTheta, kITheta, kDTheta);
        thetaController.enableContinuousInput(0, 2*Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        pathTime.stop();
        pathTime.reset();

        balanceTimer.stop();
        balanceTimer.reset();
        searchTimer.stop();
        searchTimer.reset();
        levelTime.stop();
        levelTime.reset();

        mode = Control.PLAYER;

        driveController = new PIDController(kPDrive, kIDrive, kDDrive);
        balanceController = new PIDController(kPBalance, kIBalance, kDBalance);

        balanceFound = false;
    }

    
    @Override
    public void periodic(){

        //AUTO: takes sample of path per second and adjusts robot to match path
        if (mode == Control.PATH && path != null) {
            PathPlannerTrajectory.PathPlannerState goal = ((PathPlannerTrajectory.PathPlannerState) path.sample(pathTime.get()));
            Pose2d currentRobotPose = RobotState.getInstance().getOdometryFieldToRobot();
        
            ChassisSpeeds adjustedSpeeds = controller.calculate(currentRobotPose, goal);

            setGoalChassisSpeeds(adjustedSpeeds);

            pathIsFinished();
        }

        if (mode == Control.BALANCE) {
            if (!balanceFound) {
                Pose2d currentPose = RobotState.getInstance().getOdometryFieldToRobot();
                double currentX = currentPose.getX();
                double calculate = driveController.calculate(currentX, targetStationX);
                setGoalChassisSpeeds(new ChassisSpeeds(0, calculate, 0));
                if (Math.abs(pigeon.getRoll()) > 0.2) {
                    balanceFound = true;
                    searchTimer.stop();
                    balanceTimer.reset();
                    balanceTimer.start();
                    levelTime.reset();
                    levelTime.start();
                }
            } else {
                double pitch = pigeon.getRoll();
                double dir = balanceController.calculate(pitch, 0);
                setGoalChassisSpeeds(new ChassisSpeeds(0, dir, 0));
                if (pitch != 0) {
                    levelTime.reset();
                }
            }

            balanceIsFinished();
        }

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

<<<<<<< HEAD
    
=======
    //AUTO: initializes auto
    public void driveByPath(PathPlannerTrajectory path) {

        mode = Control.PATH;

        pathTime.reset();
        pathTime.start();  

        this.path = path;
    }

    //BALANCE: initializes balance
    public void driveByBalance() {

        mode = Control.BALANCE;

        searchTimer.reset();
        searchTimer.start();

        balanceFound = false;
        
    }

    //AUTO: checks for finished path, resets conditions if true
    public boolean pathIsFinished() {
        if (pathTime.get() > path.getTotalTimeSeconds() && mode == Control.PATH) {
            mode = Control.PLAYER;
            pathTime.stop();
            pathTime.reset();
            path = null;
            return true;
        } 
        if (mode != Control.PATH) {
            pathTime.stop();
            pathTime.reset();
            path = null;
            return true;
        }
        return false;
    }

    //BALANCE: checks for finished balance, resets conditions if true
    public boolean balanceIsFinished() {
        if (((searchTimer.get() > 10 && balanceTimer.get() > 10) || (levelTime.get() > 3)) && mode == Control.BALANCE) {

            mode = Control.PLAYER;

            balanceFound = false;

            balanceTimer.stop();
            balanceTimer.reset();
            levelTime.stop();
            levelTime.reset();
            searchTimer.stop();
            searchTimer.reset();

            return true;
        } 
        if (mode != Control.BALANCE) {

            balanceFound = false;

            balanceTimer.stop();
            balanceTimer.reset();
            levelTime.stop();
            levelTime.reset();
            searchTimer.stop();
            searchTimer.reset();

            return true;
        } 
        return false;
    }

>>>>>>> 125845b (edits made!)
} 


