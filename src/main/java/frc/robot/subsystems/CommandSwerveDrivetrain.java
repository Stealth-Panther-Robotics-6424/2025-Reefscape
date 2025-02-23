package frc.robot.subsystems;

// Import necessary classes and packages from WPILib, CTRE Phoenix 6, and PathPlanner
import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * This class extends the Phoenix 6 SwerveDrivetrain class and implements the
 * Subsystem
 * interface to make it compatible with command-based FRC projects.
 * 
 * The class provides functionality for both manual and autonomous control of a
 * swerve drivetrain,
 * while also including tools for tuning the drivetrain's PID controllers using
 * SysId tests.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // Simulation configuration constants
    private static final double kSimLoopPeriod = 0.005; // 5 ms for simulation loop
    private Notifier m_simNotifier = null; // Notifier for sim thread
    private double m_lastSimTime; // Track the last simulation time

    /* Define the operator's perspective based on alliance color */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero; // Blue alliance sees forward
                                                                                         // as 0 degrees
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg; // Red alliance sees forward
                                                                                          // as 180 degrees
    private boolean m_hasAppliedOperatorPerspective = false; // Flag for whether operator perspective has been set

    /* SysId routines to characterize different aspects of the drivetrain */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    // PathPlanner configuration for autonomous driving
    RobotConfig config; // Holds PathPlanner robot configuration

    // SysId routines for characterizing translation, steer, and rotation
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate of 1 V/s
                    Volts.of(4), // Set step voltage to 4 V to prevent brownouts
                    null, // Default timeout of 10 seconds
                    // Log state with SignalLogger for analysis
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null, // No additional logic for translation
                    this // Reference to this class instance
            ));

    // SysId routine for characterizing steer (PID tuning for steer motors)
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Default ramp rate of 1 V/s
                    Volts.of(7), // Set dynamic voltage of 7 V
                    null, // Default timeout of 10 seconds
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString()) // Log state during tests
            ),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null, // No additional logic for steer
                    this // Reference to this class instance
            ));

    // SysId routine for characterizing rotation (PID tuning for rotational control)
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.of(Math.PI / 6).per(Second), // Input is in radians per second²
                    Volts.of(Math.PI), // Input is in radians per second
                    null, // Default timeout of 10 seconds
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString()) // Log state during tests
            ),
            new SysIdRoutine.Mechanism(
                    output -> {
                        // Convert radians per second to volts for SysId testing
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts)); // Log rotational rate
                    },
                    null, // No additional logic for rotation
                    this // Reference to this class instance
            ));

    // Default SysId routine to apply (starting with translation characterization)
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain with the specified constants for the
     * drivetrain and swerve modules.
     * <p>
     * This constructor initializes the underlying hardware devices, and users
     * should not initialize them manually.
     * Instead, users should use the getter methods to access the devices.
     *
     * @param drivetrainConstants Constants for the drivetrain-wide swerve drive
     *                            configuration
     * @param modules             Constants for each individual swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread(); // Start simulation thread if in simulation mode
        }
        configureAutoBuilder(); // Configure PathPlanner's AutoBuilder for autonomous driving
    }

    /**
     * Constructs a CTRE SwerveDrivetrain with the specified constants and odometry
     * update frequency.
     * <p>
     * This constructor initializes the underlying hardware devices, and users
     * should not initialize them manually.
     *
     * @param drivetrainConstants     Constants for the drivetrain-wide swerve drive
     *                                configuration
     * @param odometryUpdateFrequency Frequency to run the odometry loop (in Hz)
     * @param modules                 Constants for each individual swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread(); // Start simulation thread if in simulation mode
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain with the specified constants for odometry
     * and vision standard deviations.
     * <p>
     * This constructor initializes the underlying hardware devices, and users
     * should not initialize them manually.
     *
     * @param drivetrainConstants       Constants for the drivetrain-wide swerve
     *                                  drive configuration
     * @param odometryUpdateFrequency   Frequency to run the odometry loop (in Hz)
     * @param odometryStandardDeviation Standard deviation for odometry calculation
     * @param visionStandardDeviation   Standard deviation for vision calculation
     * @param modules                   Constants for each individual swerve module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread(); // Start simulation thread if in simulation mode
        }
    }

    /**
     * Returns a command that applies the specified control request to the swerve
     * drivetrain.
     * This allows the drivetrain to perform actions such as movement or state
     * changes.
     *
     * @param request Supplier function returning the swerve request to apply
     * @return Command to execute the request
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in a given direction for the routine
     * specified by m_sysIdRoutineToApply.
     * This is a low-speed, quasi-static test used for PID tuning.
     *
     * @param direction Direction of the SysId test
     * @return Command to execute the SysId test
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in a given direction for the routine specified by
     * m_sysIdRoutineToApply.
     * This is a high-speed, dynamic test used for PID tuning.
     *
     * @param direction Direction of the SysId test
     * @return Command to execute the SysId test
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /**
     * Periodically checks if the operator perspective should be applied, based on
     * alliance color and match state.
     * It ensures that the correct perspective (blue or red) is used for the
     * operator's control.
     */
    @Override
    public void periodic() {
        // Apply operator perspective if not already applied or if the robot is disabled
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Starts the simulation thread, which periodically updates the robot's
     * simulation state.
     * It also adjusts the PID gains to make sure the robot behaves more
     * realistically in the simulation.
     */
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds(); // Initialize simulation time

        /* Run simulation at a faster rate to improve PID tuning behavior */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /*
             * Use the measured time delta and the robot's battery voltage for simulation
             * updates
             */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod); // Start periodic simulation updates
    }

    /**
     * Configures the PathPlanner AutoBuilder for autonomous routines based on the
     * robot's current configuration.
     * This includes path following, robot speed control, and PID tuning for
     * autonomous driving.
     */
    private void configureAutoBuilder() {
        try {
            config = RobotConfig.fromGUISettings(); // Load robot configuration from GUI
        } catch (Exception e) {
            // Handle exceptions during PathPlanner config loading
            e.printStackTrace();
        }

        // Configure the AutoBuilder with the robot's state and PathPlanner
        // configuration
        AutoBuilder.configure(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::resetPose, // Consumer to reset robot pose in autonomous
                () -> getState().Speeds, // Supplier of current robot speeds
                (speeds, feedforwards) -> setControl(
                        m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                new PPHolonomicDriveController(
                        new PIDConstants(10, 0, 0), // PID constants for translation
                        new PIDConstants(7, 0, 0) // PID constants for rotation
                ),
                config, // PathPlanner robot configuration
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Flip path for Red alliance
                this // This class as the subsystem for requirements
        );
    }
}
