package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.ArmSubsystem;

public class RobotContainer {


    private ArmSubsystem wrist = new ArmSubsystem();
    private XboxController gamepad = new XboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

}
