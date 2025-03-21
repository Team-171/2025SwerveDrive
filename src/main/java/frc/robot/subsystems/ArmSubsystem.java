package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax m_arm = new SparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);

    private final SparkAbsoluteEncoder encoder;
    private double holdPosition;

    private PIDController armPidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI,
            ArmConstants.kArmD);

    public ArmSubsystem() {

        encoder = m_arm.getAbsoluteEncoder();

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig
                .smartCurrentLimit(ArmConstants.kArmMotorCurrentLimit)
                .idleMode(IdleMode.kBrake)
                .closedLoopRampRate(0.125);

        m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        holdPosition = getAbsoluteEncoderValue();
    }

    public void moveArm(double speed) {
        m_arm.set(speed);
        // holdPosition = getAbsoluteEncoderValue();
    }

    public void stopArm() {
        m_arm.stopMotor();
    }

    public void setHoldPosition(double position) {
        holdPosition = position;
    }

    // not tested
    public void holdCurrentPosition() {
        double currentPosition = getAbsoluteEncoderValue();
        SmartDashboard.putNumber("currentPositionAfterWrapper", currentPosition);

        SmartDashboard.putNumber("HoldingHoldPosition", holdPosition);
        holdPosition = MathUtil.clamp(holdPosition, ArmConstants.kArmCounterClockwiseStop, ArmConstants.kArmClockwiseStop);

        double speed = armPidController.calculate(currentPosition, holdPosition);
        speed = MathUtil.clamp(speed, -ArmConstants.kArmSpeed, ArmConstants.kArmSpeed);

        // m_arm.set(speed);
        SmartDashboard.putNumber("PIDArmSpeed", speed);
    }

    private double getAbsoluteEncoderValue() {
        double encoderValue = encoder.getPosition();
        if (encoderValue < ArmConstants.kMiddleBadBounds)
            encoderValue += 1;
        return encoderValue;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Encoder", getAbsoluteEncoderValue());
        SmartDashboard.putNumber("ArmHoldPosition", holdPosition);
    }
}
