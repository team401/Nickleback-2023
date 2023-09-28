package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmMove;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BasicDriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.Mode;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {

    private final BasicDriveSubsystem drive = new BasicDriveSubsystem();
    private ArmSubsystem arm = new ArmSubsystem();

    private Joystick leftJoystick = new Joystick(0);
    private Joystick rightJoystick = new Joystick(1);

    private CommandXboxController gamepad = new CommandXboxController(2);

    private ArmMove armIntake = new ArmMove(arm, Mode.INTAKE);
    private ArmMove armShoot = new ArmMove(arm, Mode.SHOOT);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;

    public RobotContainer() {
        drive.setDefaultCommand(
            new InstantCommand(
                () -> drive.arcadeDrive(
                        leftJoystick.getY(),
                        rightJoystick.getX()),
                drive)
        );
        configureButtonBindings();
        
    }

    private void configureButtonBindings() {
        gamepad.a()
            .whileTrue(armIntake);
        gamepad.y()
            .whileTrue(armShoot);
            
    }

    private void configureSubsystems() {
    }
}
