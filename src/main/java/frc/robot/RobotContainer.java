// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*TODO 
 *  Base Sprint 1/27/2025
 *  Base Sprint 3.0 
 *  Base Sprint 4.0
*/

// Import necessary packages for robot control, mathematical operations, and subsystem handling
package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Method;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.EndEffector;

@SuppressWarnings("unused")
public class RobotContainer {

        private ShuffleboardTab DS_MainTab = Shuffleboard.getTab("Main");
        private GenericEntry DS_CodeVersion = DS_MainTab.add("Code Version", Constants.codeVersion).getEntry();
        private GenericEntry DS_AlgeaMode = DS_MainTab.add("Algea Mode", false).getEntry();

        // Define maximum speed and angular rate based on tuner constants, converted
        // into appropriate units
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        // Create swerve drive requests for field-centric and robot-centric driving
        // modes with deadband
        private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        // Set up brake and wheel pointing functionalities for swerve drive
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        /* Path follower for autonomous control */
        // private final SendableChooser<Command> autoChooser;

        // Create a telemetry logger for monitoring robot stats
        // private final Telemetry logger = new Telemetry(MaxSpeed);

        // Joysticks for controlling the robot, one for driving and one for the button
        // board
        private final CommandJoystick joystick = new CommandJoystick(0); // My joystick
        private final CommandJoystick buttonbord = new CommandJoystick(1); // Buttonboard joystick
        private final Wrist wrist = new Wrist(); // Wrist subsystem for arm control
        private final Elevator elevator = new Elevator(); // Elevator subsystem for vertical movements
        private final EndEffector endEffector = new EndEffector(); // Intake subsystem for grabbing objects
        private final Climber climber = new Climber(); // Intake subsystem for grabbing objects

        private boolean AlgeaMode = false;

        // Instantiate the swerve drivetrain subsystem
        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // Define triggers for controlling wrist and elevator limits
        private final Trigger wristLimiter = wrist.wristLimiter();
        private final Trigger canFold = elevator.canFold();
        private final Trigger wristIntake = wrist.wristIntake();
        private final Trigger elevatorIntake = elevator.elevatorIntake();
        private final Trigger isEnabled = new Trigger(() -> DriverStation.isEnabled());
        private final Trigger isDisabled = new Trigger(() -> DriverStation.isDisabled());
        private final Trigger algeaModeEnabled = new Trigger(() -> getAlgeaMode());

        /* Some triggers related to elevator throttles (to be developed in Sprint 4) */
        /*
         * private final Trigger driveThrottleL2 = elevator.driveThrottleL2(); TODO
         * Sprint 4 beta
         * private final Trigger driveThrottleL3 = elevator.driveThrottleL3();
         * private final Trigger driveThrottleL4 = elevator.driveThrottleL4();
         * 
         * 
         * 
         * /*
         * 
         * Class Constructor
         * 
         */
        public RobotContainer() {
                // Set up autonomous command chooser using PathPlanner
                // autoChooser = AutoBuilder.buildAutoChooser();
                // SmartDashboard.putData("Auto Mode", autoChooser); // Display auto mode
                // selector on the dashboard

                // Define and register commands for the intake subsystem with different
                // behaviors
                /*
                 * NamedCommands.registerCommand("IntakeCoral", intake.IntakeCoral(() -> true,
                 * () -> false).withTimeout(3));
                 * NamedCommands.registerCommand("StopIntake", intake.IntakeCoral(() -> false,
                 * () -> false).withTimeout(0.1));
                 * NamedCommands.registerCommand("SpitCoral", intake.IntakeCoral(() -> false, ()
                 * -> true).withTimeout(3));
                 */

                configureBindings(); // Configure control bindings for robot functions
        }

        public boolean getAlgeaMode() {
                return this.AlgeaMode;

        }

        public void DS_Update() {
                this.DS_AlgeaMode.setBoolean(this.getAlgeaMode());

        }

        // Configure the control bindings for the robot's subsystems and commands
        private void configureBindings() {

                buttonbord.button(8).onTrue(Commands.runOnce(() -> this.AlgeaMode = !this.AlgeaMode));

                // Drivetrain control for swerve drive based on joystick input
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> fieldCentricDrive
                                                .withVelocityX(-joystick.getRawAxis(1) * MaxSpeed * throttle(3)
                                                                * elevator.elevatorThrottle()) // Drive
                                                                                               // forward
                                                                                               // with
                                                                                               // negative
                                                                                               // Y
                                                                                               // (forward)
                                                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed * throttle(3)
                                                                * elevator.elevatorThrottle()) // Drive
                                                                                               // left
                                                                                               // with
                                                                                               // negative
                                                                                               // X
                                                                                               // (left)
                                                .withRotationalRate(
                                                                -joystick.getRawAxis(4) * MaxAngularRate * throttle(3)
                                                                                * elevator.elevatorThrottle()) // Drive
                                                                                                               // counterclockwise
                                                                                                               // with
                                                                                                               // negative
                                                                                                               // X
                                                                                                               // (left)
                                ));

                // Button bindings for switching to robot-centric control mode
                joystick.button(8).toggleOnTrue(drivetrain.applyRequest(() -> robotCentricDrive
                                .withVelocityX(-joystick.getRawAxis(1) * MaxSpeed * throttle(3)
                                                * elevator.elevatorThrottle()) // Drive forward with
                                // negative Y (forward)
                                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed * throttle(3)
                                                * elevator.elevatorThrottle()) // Drive left with
                                                                               // negative X (left)
                                .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate * throttle(3)
                                                * elevator.elevatorThrottle())) // Drive
                                                                                // counterclockwise
                                                                                // with
                                                                                // negative
                                                                                // X (left)
                );

                // Joystick button to reset field-centric heading
                joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Joystick button to apply the brake to stop all swerve drive modules
                joystick.button(4).whileTrue(drivetrain.applyRequest(() -> brake));

                // Button to point the wheels in a specific direction based on joystick input
                joystick.button(3).whileTrue(drivetrain.applyRequest(
                                () -> point.withModuleDirection(
                                                new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0)))));

                // Run SysId routines when specific button combinations are pressed
                /*
                 * joystick.button(7).and(joystick.button(4)).whileTrue(drivetrain.sysIdDynamic(
                 * Direction.kForward));
                 * joystick.button(7).and(joystick.button(1)).whileTrue(drivetrain.sysIdDynamic(
                 * Direction.kReverse));
                 * joystick.button(8).and(joystick.button(4)).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kForward));
                 * joystick.button(8).and(joystick.button(1)).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kReverse));
                 */
                // Register telemetry logging for the drivetrain subsystem
                // drivetrain.registerTelemetry(logger::telemeterize);

                // Intake command bindings, such as when to turn on intake or control trays
                endEffector.setDefaultCommand(endEffector.nothing());
                algeaModeEnabled.negate().and(elevator.elevatorIntake().and(wrist.wristIntake()))
                                .whileTrue(endEffector.IntakeCoral());
                algeaModeEnabled.negate().and(buttonbord.button(5)).whileTrue(endEffector.IntakeCoral());
                algeaModeEnabled.negate().and(buttonbord.button(2)).whileTrue(endEffector.manualIntake());
                algeaModeEnabled.negate().and(buttonbord.button(3)).whileTrue(endEffector.manualBackFeed());
                algeaModeEnabled.and(buttonbord.button(5)).whileTrue(endEffector.IntakeAlgea());

                algeaModeEnabled.and(buttonbord.button(5).negate()).and(buttonbord.button(2)
                                .negate())
                                .whileTrue(endEffector.HoldAlgea());

                // buttonbord.button(6).whileTrue(endEffector.HoldAlgea());
                algeaModeEnabled.and(buttonbord.button(2)).and(buttonbord.button(5).negate())
                                .whileTrue(endEffector.ShootAlgea());

                // Commands for wrist and elevator control using buttonboard inputs
                wrist.setDefaultCommand(wrist.WristPIDCommandDefault(() -> canFold.getAsBoolean()));
                buttonbord.axisGreaterThan(0, 0.1).or(buttonbord.axisLessThan(0, -0.1))
                                .whileTrue(wrist.ManualWrist(() -> buttonbord.getRawAxis(0),
                                                () -> canFold.getAsBoolean()));

                // Elevator control with wrist limit consideration
                elevator.setDefaultCommand((elevator.elevatorPIDCommandDefault(() -> wristLimiter.getAsBoolean())));
                (wristLimiter).and((buttonbord.axisGreaterThan(1, 0.1).or(buttonbord.axisLessThan(1, -0.1)))
                                .whileTrue(elevator.ManualElevator(() -> buttonbord.getRawAxis(1),
                                                () -> wristLimiter.getAsBoolean())));

                // Commands to preserve position when enabled
                this.isEnabled.onTrue(elevator.startCommand(wristLimiter));
                this.isEnabled.onTrue(wrist.startWristCommand());
                this.isDisabled.onTrue(elevator.EndCommand(wristLimiter));

                // Wrist and elevator commands for specific positions, triggered by button
                // presses
                algeaModeEnabled.negate().and(buttonbord.button(1))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL4(wristLimiter),
                                                wrist.WristL4(() -> canFold.getAsBoolean())));
                algeaModeEnabled.negate().and(buttonbord.button(4))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL3(wristLimiter),
                                                wrist.WristL3(() -> canFold.getAsBoolean())));
                algeaModeEnabled.negate().and(buttonbord.button(7))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL2(wristLimiter),
                                                wrist.WristL2(() -> canFold.getAsBoolean())));
                algeaModeEnabled.negate().and(buttonbord.button(11))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorL1(wristLimiter),
                                                wrist.WristL1(() -> canFold.getAsBoolean())));
                // Algea Positions//
                algeaModeEnabled.and(buttonbord.button(7))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorA1(wristLimiter),
                                                wrist.WristA1(() -> canFold.getAsBoolean())));

                algeaModeEnabled.and(buttonbord.button(4))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()),
                                                elevator.ElevatorA2(wristLimiter),
                                                wrist.WristA2(() -> canFold
                                                                .getAsBoolean())));

                algeaModeEnabled.and(buttonbord.button(1))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()), elevator.ElevatorBarge(wristLimiter),
                                                wrist.WristBarge(() -> canFold.getAsBoolean())));

                algeaModeEnabled.and(buttonbord.button(11))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()),
                                                elevator.ElevatorProcessor(wristLimiter),
                                                wrist.WristProcessor(() -> canFold
                                                                .getAsBoolean())));

                algeaModeEnabled.and(buttonbord.button(9))
                                .onTrue(Commands.sequence(wrist.WristSafety(
                                                () -> canFold.getAsBoolean()),
                                                elevator.ElevatorProcessor(wristLimiter),
                                                wrist.WristProcessor(() -> canFold
                                                                .getAsBoolean())));

                joystick.button(5).or(joystick.button(6))
                                .onTrue(climber.ManualClimber(() -> joystick.button(6).getAsBoolean(),
                                                () -> joystick.button(5).getAsBoolean()));

                buttonbord.button(12)
                                .onTrue(Commands.parallel(wrist.ExitState(() -> canFold.getAsBoolean()),
                                                elevator.ExitState(wristLimiter)));

                // Buttonboard button 8 toggles manual tray control for the intake
                buttonbord.button(10)
                                .toggleOnTrue(climber.TrayManual());

        }

        private double throttle(int throttle_axis) {
                return ((1 - (0.8 * Math.pow(this.joystick.getRawAxis(throttle_axis), 0.5))));
        }

        // Autonomous command that is selected based on the chosen auto mode
        public Command getAutonomousCommand() {
                /* Run the path selected from the auto chooser */
                return null; // autoChooser.getSelected();
        }
}