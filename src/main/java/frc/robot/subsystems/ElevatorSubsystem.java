package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.utils.EncoderUtils.EncoderPosition;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax m_leftLift = new SparkMax(ElevatorConstants.kLeftLiftCanID, MotorType.kBrushless);
    private final SparkMax m_rightLift = new SparkMax(ElevatorConstants.kRightLiftCanID, MotorType.kBrushless);

    private final DutyCycleEncoder encoder;
    private final Encoder relativeEncoder;
    private final EncoderPosition encoderPosition;
    private DigitalInput digitalInput;
    private DutyCycle sensor;
    private double distance;

    private double holdPosition; // in rotation ticks when set for encoder

    private final PIDController pid;

    private SparkMaxConfig leftLiftConfig = new SparkMaxConfig();
    private SparkMaxConfig rightLiftConfig = new SparkMaxConfig();
    private SparkMaxConfig coastModeConfig = new SparkMaxConfig();

    public ElevatorSubsystem() {
        // Get if we want to reset encoder
        SmartDashboard.putBoolean("Reset Encoder?", false);

        digitalInput = new DigitalInput(4);
        sensor = new DutyCycle(digitalInput);

        // Encoder
        encoder = new DutyCycleEncoder(1);
        relativeEncoder = new Encoder(2, 3, false, EncodingType.k1X);
        relativeEncoder.setDistancePerPulse(0.00048828125);

        encoderPosition = new EncoderPosition(encoder.get());

        //holdPosition = encoder.get();
        //holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopEncoder, ElevatorConstants.kHighStopEncoder);
        holdPosition = distance;
        holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopDistanceSensor, ElevatorConstants.kHighStopDistanceSensor);

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
        coastModeConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(ElevatorConstants.kElevatorMotorCurrentLimit)
                .closedLoopRampRate(ElevatorConstants.kElevatorMotorRampRate);

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
        //double currentPosition = encoder.get();
        double currentPosition = distance;
        if (currentPosition != 0) {

            //holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopEncoder, ElevatorConstants.kHighStopEncoder);
            holdPosition = MathUtil.clamp(holdPosition, ElevatorConstants.kLowStopDistanceSensor, ElevatorConstants.kHighStopDistanceSensor);

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
        //double currentPosition = encoder.get();
        double currentPosition = distance;
        
        //if (ElevatorConstants.kLowStopEncoder >= currentPosition || ElevatorConstants.kHighStopEncoder <= currentPosition) {
        if (ElevatorConstants.kLowStopDistanceSensor >= currentPosition || ElevatorConstants.kHighStopDistanceSensor <= currentPosition) {
            speed = 0;
        }
        elevatorMove(speed);

        //holdPosition = encoder.get();
        holdPosition = distance;
    }

    public void resetEncoder() {
        relativeEncoder.reset();
    }

    public void getEncoderValue() {
        SmartDashboard.putNumber("encoderSavePos", encoder.get());
    }

    @Override
    public void periodic() {
        encoderPosition.updatePosition(encoder.get());
        SmartDashboard.putNumber("encoder", encoder.get());
        SmartDashboard.putNumber("Relative Encoder", relativeEncoder.getDistance());
        // distance is non-linear and not a real measurement of distance but is good enough and consistent
        distance = (((sensor.getHighTimeNanoseconds() / 1000) * 0.3 / 2) - 150) * 2;

        SmartDashboard.putNumber("dutyCycleTime", sensor.getHighTimeNanoseconds());
        SmartDashboard.putNumber("distance", distance);

        if(DriverStation.isDisabled()){
            if(SmartDashboard.getBoolean("Reset Encoder?", false)){
                relativeEncoder.reset();
                SmartDashboard.putBoolean("Reset Encoder?", false);
            }
            if(m_leftLift.configAccessor.getIdleMode() == IdleMode.kBrake){
                m_leftLift.configure(coastModeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                m_rightLift.configure(coastModeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            }
        }else if(DriverStation.isEnabled() && m_leftLift.configAccessor.getIdleMode() == IdleMode.kCoast){
            m_leftLift.configure(leftLiftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            m_rightLift.configure(rightLiftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }   
    }
}
