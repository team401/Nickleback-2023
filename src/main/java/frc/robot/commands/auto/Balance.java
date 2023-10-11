package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.PigeonInterface;

public class Balance extends CommandBase {

    private DriveSubsystem drive;
    private PigeonInterface pigeon = new PigeonInterface();

    private PIDController driveController, balanceController;

    private double targetStationX = 0; //TODO: get location of targetStationX

    private Timer searchTimer = new Timer();
    private Timer balanceTimer = new Timer();

    private boolean balanceFound;

    //TODO: get pid values
    private double kPDrive = 0, kIDrive = 0, kDDrive = 0, kPBalance = 0, kIBalance = 0, kDBalance = 0;

    public Balance(DriveSubsystem drive) {
        this.drive = drive;
        driveController = new PIDController(kPDrive, kIDrive, kDDrive);
        balanceController = new PIDController(kPBalance, kIBalance, kDBalance);

    }

    @Override
    public void initialize() {
        searchTimer.reset();
        searchTimer.start();

        balanceFound = false;

        SmartDashboard.putNumber("kP Drive", kPDrive);
        SmartDashboard.putNumber("kI Drive", kIDrive);
        SmartDashboard.putNumber("kD Drive", kDDrive);

        SmartDashboard.putNumber("kP Balance", kPBalance);
        SmartDashboard.putNumber("kI Balance", kIBalance);
        SmartDashboard.putNumber("kD Balance", kDBalance);

    }

    @Override
    public void execute() {
        if (!balanceFound) {
            Pose2d currentPose = RobotState.getInstance().getOdometryFieldToRobot();
            double currentX = currentPose.getX();
            double calculate = driveController.calculate(currentX, targetStationX);
            drive.setGoalChassisSpeeds(new ChassisSpeeds(0, calculate, 0));
            if (Math.abs(pigeon.getRoll()) > 0.2) {
                balanceFound = true;
                searchTimer.stop();
                balanceTimer.reset();
                balanceTimer.start();
            }
        } else {
            double pitch = pigeon.getRoll();
            double dir = balanceController.calculate(pitch, 0);
            drive.setGoalChassisSpeeds(new ChassisSpeeds(0, dir, 0));
        }


    
        SmartDashboard.getNumber("kP Drive", kPDrive);
        SmartDashboard.getNumber("kI Drive", kIDrive);
        SmartDashboard.getNumber("kD Drive", kDDrive);

        SmartDashboard.getNumber("kP Balance", kPBalance);
        SmartDashboard.getNumber("kI Balance", kIBalance);
        SmartDashboard.getNumber("kD Balance", kDBalance);

    }

    @Override
    public boolean isFinished() {
        return (searchTimer.get() > 10 && balanceTimer.get() > 10);
    }

<<<<<<< HEAD
    @Override
    public void end(boolean interupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
=======
    /*@Override
    public void end() {
        //TODO: stop drivetrain
    }*/
>>>>>>> 7d738b7 (waiting on swerve and odometry :D)

}

