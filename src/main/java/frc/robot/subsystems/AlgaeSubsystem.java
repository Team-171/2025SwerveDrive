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
    private final SparkMax m_arm = new SparkMax(AlgaeConstants.kArmMotorCanId, MotorType.kBrushless);
    private final SparkMax m_roller = new SparkMax(AlgaeConstants.kRollerMotorCanId, MotorType.kBrushless);

    private final AbsoluteEncoder encoder;

    public AlgaeSubsystem() {

        encoder = m_arm.getAbsoluteEncoder();

        SparkMaxConfig armConfig = new SparkMaxConfig();
        SparkMaxConfig rollerConfig = new SparkMaxConfig();

        armConfig
            .smartCurrentLimit(AlgaeConstants.kArmMotorCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.125);

        rollerConfig
            .smartCurrentLimit(AlgaeConstants.kRollerMotorCurrentLimit)
            .idleMode(IdleMode.kBrake);

        m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_roller.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveArm(double speed) {
        m_arm.set(speed);
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public void moveRoller(double speed) {
        m_roller.set(speed);
    }

    public void stopRoller() {
        m_roller.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder", encoder.getPosition());
    }
}
