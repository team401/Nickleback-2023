package frc.robot.commands.drive;


import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;

public class SwerveDriveCommand extends CommandBase{
    private final DriveSubsystem drive;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;
    private final DoubleSupplier omegaPercent;
    private final boolean fieldRelative;

    public SwerveDriveCommand(DriveSubsystem drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, boolean fieldRelative) {
        this.drive = drive;
        this.xPercent = xPercent;
        this.yPercent = yPercent;
        this.omegaPercent = omegaPercent;
        this.fieldRelative = fieldRelative;

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xMPerS = processJoystickInputs(xPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed;
        double yMPerS = processJoystickInputs(yPercent.getAsDouble(), false) * DriveConstants.maxDriveSpeed;
        double omegaRadPerS = processJoystickInputs(omegaPercent.getAsDouble(), true) * DriveConstants.maxTurnRate;

        //Convert to field relative speeds
        ChassisSpeeds targetSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xMPerS, yMPerS, omegaRadPerS, drive.getRotation())
            : new ChassisSpeeds(xMPerS, yMPerS, omegaRadPerS);

        drive.setGoalChassisSpeeds(targetSpeeds);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private double processJoystickInputs(double value, boolean square) {
        double scaledValue = 0.0;
        double deadband = DriveConstants.driveJoystickDeadbandPercent;
        if (Math.abs(value) > deadband) {
            scaledValue = (Math.abs(value) - deadband) / (1 - deadband);
            if (square) {
                scaledValue = Math.copySign(scaledValue * scaledValue, value);
            } else {
                scaledValue = Math.copySign(scaledValue, value);
            }
        }
        return scaledValue;
    }
}
