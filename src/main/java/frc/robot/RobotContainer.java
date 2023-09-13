package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.TankDrive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ArmMove;
import frc.robot.commands.Intake;

public class RobotContainer {

    private final Joystick leftStick = new Joystick(0);
    private final Joystick rightStick = new Joystick(1);
    private final XboxController gamepad = new XboxController(2);
    private ArmSubsystem wrist = new ArmSubsystem();
    private Intake intake = new Intake();

    //private XboxController gamepad = new XboxController(0);

    SendableChooser<String> autoChooser = new SendableChooser<String>();
    private Command activeAutoCommand = null;
    private String activeAutoName = null;
    }

    public RobotContainer() {
        drive.setDefaultCommand(new TankDrive(drive,
                () -> leftStick.getRawAxis(1),
                () -> rightStick.getRawAxis(0)));
        configureButtonBindings();
        
    }

    private void configureButtonBindings() {
        new JoystickButton(gamepad, Button.kY.value)
            .onTrue(new InstantCommand(intake::shoot))
            .onFalse(new InstantCommand(intake::stop));
        new JoystickButton(gamepad, Button.kX.value)
            .onTrue(new InstantCommand(intake::spit))
            .onFalse(new InstantCommand(intake::stop));
            
    }

    private void configureSubsystems() {
    }
}
