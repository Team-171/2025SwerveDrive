package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase{
    private final SparkMax m_roller = new SparkMax(AlgaeConstants.kRollerMotorCanId, MotorType.kBrushless);

    public AlgaeSubsystem() {

        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        rollerConfig
            .smartCurrentLimit(AlgaeConstants.kRollerMotorCurrentLimit)
            .idleMode(IdleMode.kBrake);

        m_roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveRoller(double speed) {
        m_roller.set(speed);
    }

    public void stopRoller() {
        m_roller.stopMotor();
    }

    @Override
    public void periodic(){
    }
}
