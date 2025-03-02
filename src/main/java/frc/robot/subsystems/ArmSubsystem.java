package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax m_arm = new SparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);

    private final AbsoluteEncoder encoder;

    public ArmSubsystem() {

        encoder = m_arm.getAbsoluteEncoder();

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig
            .smartCurrentLimit(ArmConstants.kArmMotorCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .closedLoopRampRate(0.125);

        m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void moveArm(double speed) {
        m_arm.set(speed);
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder", encoder.getPosition());
    }
}
