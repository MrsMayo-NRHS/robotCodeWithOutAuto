package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.PIDGains;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    /** The main driving motor. Initializes as the left one */
    private final CANSparkMax leadMotor;
    /** The following motor. Mimics all input to the {@link #leadMotor}. Initializes as the right one */
    private final CANSparkMax followerMotor;
    private final AbsoluteEncoder encoder;
    private final SparkPIDController controller;
    private double setpoint;

    private TrapezoidProfile profile;
    private final Timer timer;
    private TrapezoidProfile.State startState;
    private TrapezoidProfile.State endState;

    private TrapezoidProfile.State targetState;
    private double feedforward;
    private double manualValue;

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    /**
     * The Singleton instance of this ArmSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static ArmSubsystem INSTANCE = new ArmSubsystem();

    /**
     * Returns the Singleton instance of this ArmSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code ArmSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static ArmSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this ArmSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private ArmSubsystem() {

        leadMotor = new CANSparkMax(ArmConstants.leftCanID, MotorType.kBrushless);
        leadMotor.setInverted(ArmConstants.leftInverted);
        leadMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
        leadMotor.setIdleMode(IdleMode.kBrake);
        leadMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leadMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        leadMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.softLimitForward);
        leadMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.softLimitReverse);

        followerMotor = new CANSparkMax(ArmConstants.rightCanID, MotorType.kBrushless);
        followerMotor.setInverted(ArmConstants.rightInverted);
        followerMotor.setSmartCurrentLimit(ArmConstants.currentLimit);
        followerMotor.setIdleMode(IdleMode.kBrake);
        followerMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        followerMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        followerMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ArmConstants.softLimitForward);
        followerMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) ArmConstants.softLimitReverse);

        // set up the motor encoder including conversion factors to convert to radians and radians per second for position and velocity
        encoder = ((ArmConstants.encoderConnectedToLeftSparkMax) // Depending on which motor controller the encoder is connected to,
                ? leadMotor : followerMotor) // we get the encoder from that one
                .getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(ArmConstants.positionFactor);
        encoder.setVelocityConversionFactor(ArmConstants.velocityFactor);
        encoder.setZeroOffset(ArmConstants.armAbsoluteEncoderZeroOffset);

        controller = leadMotor.getPIDController();
        PIDGains.setSparkMaxGains(controller, ArmConstants.armPositionGains);

        followerMotor.follow(leadMotor, // SUPER IMPORTANT: Tells the following (right) motor to mirror the left motor
                ArmConstants.rightInverted != ArmConstants.leftInverted); // This includes the direction of rotation,
                                                                        // So we have to set the inversion depending on whether
                                                                        // it is inverted relative to the left motor

        leadMotor.burnFlash();
        followerMotor.burnFlash();

        setpoint = 0.0 + ArmConstants.armAbsoluteEncoderZeroOffset; // At initialization, it will go to the encoder's 0 setpoint

        timer = new Timer();
        timer.start();

        updateMotionProfile();

        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Arm Subsystem");
        builder.addDoubleProperty("Raw Absolute Encoder Position", () -> encoder.getPosition() + ArmConstants.armAbsoluteEncoderZeroOffset, null);
        builder.addDoubleProperty("Adjusted Absolute Encoder Position",
                encoder::getPosition, null);
        builder.addDoubleProperty("Setpoint", () -> setpoint, this::setTargetPosition);
        builder.addDoubleProperty("Manual Power", () -> manualValue, this::runManual);
    }

    /**
     * Sets the target position and updates the motion profile if the target position changed.
     * @param _setpoint The new target position in radians.
     */
    public void setTargetPosition(double _setpoint) {
        if (_setpoint != setpoint) {
            setpoint = _setpoint;
            updateMotionProfile();
        }
    }

    /**Update the motion profile variables based on the current setpoint and the pre-configured motion constraints.*/
    private void updateMotionProfile() {
        startState = new TrapezoidProfile.State(encoder.getPosition(), encoder.getVelocity());
        endState = new TrapezoidProfile.State(setpoint, 0.0);
        profile = new TrapezoidProfile(ArmConstants.armMotionConstraint);
        timer.reset();
    }

    /**
     * Drives the arm to a position using a trapezoidal motion profile.
     * This function is usually wrapped in a {@code RunCommand} which runs it repeatedly while the command is active.
     * <p>
     * This function updates the motor position control loop using a setpoint from the trapezoidal motion profile.
     * The target position is the last set position with {@code setTargetPosition}.
     */
    public void runAutomatic() {
        double elapsedTime = timer.get();
        if (profile.isFinished(elapsedTime)) {
            targetState = new TrapezoidProfile.State(setpoint, 0.0);
        } else {
            targetState = profile.calculate(elapsedTime, startState, endState);
        }

        feedforward =
                ArmConstants.armFeedforward.calculate(
                        encoder.getPosition() + ArmConstants.armAbsoluteEncoderZeroOffset, targetState.velocity);
        controller.setReference(
                targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
    }

    /**
     * Drives the arm using the provided power value (usually from a joystick).
     * This also adds in the feedforward value which can help counteract gravity.
     * @param _power The motor power to apply.
     */
    public void runManual(double _power) {
        // reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and
        // passively
        setpoint = encoder.getPosition();
        updateMotionProfile();
        // update the feedforward variable with the newly zero target velocity
        feedforward =
                ArmConstants.armFeedforward.calculate(
                        encoder.getPosition() + ArmConstants.armAbsoluteEncoderZeroOffset, targetState.velocity);
        // set the power of the motor
        leadMotor.set(_power + (feedforward / 12.0));
        manualValue = _power; // this variable is only used for logging or debugging if needed
    }

    @Override
    public void periodic() { // This method will be called once per scheduler run
    }
}

