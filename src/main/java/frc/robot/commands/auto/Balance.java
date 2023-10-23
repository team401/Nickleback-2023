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
    

    public Balance(DriveSubsystem drive) {
        this.drive = drive;
        

    }

    @Override
    public void initialize() {
        drive.driveByBalance();
    }

    @Override
    public boolean isFinished() {
        return drive.balanceIsFinished();
    }

    @Override
    public void end(boolean interupted) {
        drive.setGoalChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }

}

