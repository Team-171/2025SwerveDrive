package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.RelativeEncoder;
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
    private final RelativeEncoder relativeEncoder;
    private double holdPosition;

    private PIDController armPidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);

    public ArmSubsystem() {

        encoder = m_arm.getAbsoluteEncoder();
        relativeEncoder = m_arm.getEncoder();

        SparkMaxConfig armConfig = new SparkMaxConfig();

        armConfig
                .smartCurrentLimit(ArmConstants.kArmMotorCurrentLimit)
                .idleMode(IdleMode.kBrake)
                .closedLoopRampRate(0.125);

        m_arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // holdPosition = getAbsoluteEncoderValue();
        holdPosition = relativeEncoder.getPosition();
    }

    public void moveArm(double speed) {
        m_arm.set(speed);
        // holdPosition = getAbsoluteEncoderValue();
    }

    public void manualMoveArm(double speed) {
        m_arm.set(speed);
        holdPosition = relativeEncoder.getPosition();
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

        double target = MathUtil.clamp(holdPosition, ArmConstants.kArmCounterClockwiseStop, ArmConstants.kArmClockwiseStop);
        SmartDashboard.putNumber("targetArmPosition", target);
        
        double speed = armPidController.calculate(currentPosition, holdPosition);
        speed = MathUtil.clamp(speed, -ArmConstants.kArmSpeed, ArmConstants.kArmSpeed);

        m_arm.set(-speed);
        SmartDashboard.putNumber("PIDArmSpeed", -speed);
    }

    public void manualHoldCurrentPosition() {
        double currentPosition = relativeEncoder.getPosition();

        double speed = armPidController.calculate(currentPosition, holdPosition);
        speed = MathUtil.clamp(speed, -ArmConstants.kArmSpeed, ArmConstants.kArmSpeed);

        m_arm.set(-speed);
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
