package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.EncoderUtils.EncoderPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_leftLift = new SparkMax(ElevatorConstants.kLeftLiftCanID, MotorType.kBrushless);
    private final SparkMax m_rightLift = new SparkMax(ElevatorConstants.kRightLiftCanID, MotorType.kBrushless);

    private final SparkAbsoluteEncoder encoder;
    private final EncoderPosition encoderPosition;

    private double holdPosition; // in rotation ticks

    private final PIDController pid;

    public ElevatorSubsystem() {

        // Elevator Motors
        SparkMaxConfig leftLiftConfig = new SparkMaxConfig();
        SparkMaxConfig rightLiftConfig = new SparkMaxConfig();

        // Encoder
        encoder = m_rightLift.getAbsoluteEncoder();
        encoderPosition = new EncoderPosition(encoder.getPosition());

        holdPosition = encoder.getPosition();
        holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStop, ElevatorConstants.kHighStop);

        pid = new PIDController(1, 0, 0);

        leftLiftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate);
        rightLiftConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate)
                .inverted(true);

        m_leftLift.configure(leftLiftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_rightLift.configure(rightLiftConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void elevatorMove(double speed) {
        m_leftLift.set(speed);
        m_rightLift.set(speed);
    }

    public void stopElevator() {
        m_leftLift.stopMotor();
        m_rightLift.stopMotor();
    }

    public void holdCurrentPosition() {
        double currentPosition = encoderPosition.getPositionWithRollover(encoder.getPosition());
        if (currentPosition != 0) {

            holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStop, ElevatorConstants.kHighStop);

            double speed = pid.calculate(currentPosition, holdPosition);
            speed = MathUtil.clamp(speed, -ElevatorConstants.kElevatorSpeed, ElevatorConstants.kElevatorSpeed);

            elevatorMove(speed);
        } else {
            stopElevator();
        }
    }

    public void setHoldPosition(double position) {
        holdPosition = position;
    }

    public void manualMoveArm(double speed) {
        double currentPosition = encoderPosition.getPositionWithRollover(encoder.getPosition());
        if(ElevatorConstants.kLowStop >= currentPosition || ElevatorConstants.kHighStop <= currentPosition){
            speed = 0;
        }
        elevatorMove(speed);

        holdPosition = encoderPosition.getPositionWithRollover(encoder.getPosition());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("encoder", encoder.getPosition());
        encoderPosition.updatePosition(encoder.getPosition());
    }
}
