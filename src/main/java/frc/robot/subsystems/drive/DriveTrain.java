package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class DriveTrain {

    private final TalonFX driveMotor;
    private final TalonFX rotationMotor;
    private final CANCoder rotationEncoder;
    private final double initialOffsetRadians;

    public class Robot {
        MotorController m_frontLeft = new PWMVictorSPX(1);
        MotorController m_rearLeft = new PWMVictorSPX(2);
        MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
     
        MotorController m_frontRight = new PWMVictorSPX(3);
        MotorController m_rearRight = new PWMVictorSPX(4);
        MotorControllerGroup m_right = new MotorControllerGroup(m_frontRight, m_rearRight);
     
        DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
      }
    
}