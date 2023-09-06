package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveTime extends CommandBase {

    private final DriveSubsystem drive;
    private final double timeS;
    private final double forward;
    private double desiredAngle;

    private final Timer timer = new Timer();

	private final PIDController controller = new PIDController(0.1, 0, 0);

    public DriveTime(DriveSubsystem drive, double TimeS, boolean foward){

        this.drive = drive;
        this.timeS = timeS;
        this.forward = forward;

        addRequirements(drive);

    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        this.desiredAngle = drive.GetDriveAngle();
    }

    @Override
    public void excute(){
        double rot = -controller.calculate(drive.GetDriveAngle(), desiredAngle);
        drive.arcadeDrive(DriveConstants.autoDrivePercent * -forward, rot);
        SmartDashboard.putNumber("Rot", drive.GetDriveAngle());
		SmartDashboard.putNumber("DesRot", desiredAngle);

    }

    @Override
    public boolean isFinished(){
        return timer.hasElapsed(timeS);

    }

    @Override
    public void end(boolean isFinished){
        drive.arcadeDrive(0, 0);
    }






}