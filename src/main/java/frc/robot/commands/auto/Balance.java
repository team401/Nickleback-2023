package frc.robot.commands.auto;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class Balance extends CommandBase {

    private DriveSubsystem drive;
    private Pigeon2 pigeon = new Pigeon2(0);

    private PIDController driveController, balanceController;

    private double targetStationX = 0;

    private Timer searchTimer = new Timer();
    private Timer balanceTimer = new Timer();

    private boolean balanceFound;

    public Balance(DriveSubsystem drive) {
        this.drive = drive;
        driveController = new PIDController(0, 0, 0);
        balanceController = new PIDController(0, 0, 0);

    }

    @Override
    public void initialize() {
        searchTimer.reset();
        searchTimer.start();

        balanceFound = false;

    }

    @Override
    public void execute() {
        if (!balanceFound) {
            Pose2d currentPose = new Pose2d(); //get current pos
            double currentX = currentPose.getX();
            double calculate = driveController.calculate(currentX, targetStationX);
            //apply power to wheels 
            if (Math.abs(pigeon.getRoll()) < 0.2) {
                balanceFound = true;
                searchTimer.stop();
                balanceTimer.reset();
                balanceTimer.start();
            }
        } else {
            double pitch = pigeon.getRoll();
            double dir = balanceController.calculate(pitch, 0);
            //apply dir to swerve
        }
    }

    @Override
    public boolean isFinished() {
        return (searchTimer.get() > 10 && balanceTimer.get() > 10);
    }

    /*@Override
    public void end() {
        //stop drivetrain
    }*/

}

