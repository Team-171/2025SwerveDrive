package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
    private final SparkMax m_roller1 = new SparkMax(AlgaeConstants.kRoller1MotorCanId, MotorType.kBrushless);
    private final SparkMax m_roller2 = new SparkMax(AlgaeConstants.kRoller2MotorCanId, MotorType.kBrushless);
    
    public AlgaeSubsystem() {

        SparkMaxConfig roller1Config = new SparkMaxConfig();
        SparkMaxConfig roller2Config = new SparkMaxConfig();

        roller1Config
            .smartCurrentLimit(AlgaeConstants.kRollerMotorCurrentLimit)
            .idleMode(IdleMode.kBrake);
        roller2Config
            .smartCurrentLimit(AlgaeConstants.kRollerMotorCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        m_roller1.configure(roller1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_roller2.configure(roller2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveRoller(double speed) {
        m_roller1.set(speed);
        m_roller2.set(speed);
    }

    public void stopRoller() {
        m_roller1.stopMotor();
        m_roller2.stopMotor();
    }
}
