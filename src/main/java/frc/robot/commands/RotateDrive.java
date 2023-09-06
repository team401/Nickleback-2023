package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class RotateDrive extends CommandBase{

    private final DriveSubsystem drive;
	private final double desiredAngle;

	private final PIDController controller = new PIDController(0.01, 0, 0);

	private final Timer timer = new Timer();

	public RotateDrive(DriveSubsystem drive, double desiredAngle) {
		this.drive = drive;
		this.desiredAngle = desiredAngle;

		addRequirements(drive);
	}

    @Override
    
    public void excute(){

        SmartDashboard.putNumber("DesiredAngle", desiredAngle);
        SmartDashboard.putNumber("Angle", drive.GetDriveAngle());

        double output = -controller.calculate(drive.GetDriveAngle(), desiredAngle);
		drive.arcadeDrive(0, output);

		if (Math.abs(drive.GetDriveAngle()-desiredAngle) > 10)
		{
			timer.reset();
			timer.start();
		}
    }

    @Override
	public boolean isFinished() {
		return timer.hasElapsed(0.5);
	}

	@Override
	public void end(boolean isFinished) {
		drive.arcadeDrive(0, 0);
	}

}