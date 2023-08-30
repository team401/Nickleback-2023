package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.arm.WristSubsystem;

public class RobotContainer {


    private WristSubsystem wrist = new WristSubsystem();
    private XboxController gamepad = new XboxController(0);

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {

    }

}
