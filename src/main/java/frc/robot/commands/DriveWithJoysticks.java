package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;


public class DriveWithJoysticks extends Command {
    private final DriveSubsystem drive;
    private final DoubleSupplier xPercent;
    private final DoubleSupplier yPercent;
    private final DoubleSupplier omegaPercent;
    private final boolean fieldRelative;

    public DriveWithJoysticks(DriveSubsystem drive, DoubleSupplier xPercent, DoubleSupplier yPercent, DoubleSupplier omegaPercent, 
            boolean fieldRelative){
        this.drive = drive;
        this.xPercent = xPercent;
        this.yPercent = yPercent;
        this.omegaPercent = omegaPercent;
        this.fieldRelative = fieldRelative;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

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

    @Override
    public void end(boolean interrupted) {
    }



}
