// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*TODO 
 *  Base Sprint 1/27/2025
 *  Base Sprint 3.0 
*/

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import javax.sound.midi.Sequence;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    
  
     /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0); // My joystick
    private final CommandJoystick buttonbord = new CommandJoystick(1);
    private final Wrist wrist = new Wrist();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Trigger wristLimiter = wrist.wristLimiter();
    private final Trigger isEnabled = new Trigger(()->DriverStation.isEnabled());

    

    /*private final Trigger driveThrottleL2 = elevator.driveThrottleL2(); TODO Sprint 4 beta
    private final Trigger driveThrottleL3 = elevator.driveThrottleL3(); 
    private final Trigger driveThrottleL4 = elevator.driveThrottleL4();



/*

Class Constructor

*/ 
public RobotContainer() {
         autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);

      
         NamedCommands.registerCommand("IntakeCoral", intake.IntakeCoral(()->true, ()->false).withTimeout(3));
        NamedCommands.registerCommand("StopIntake", intake.IntakeCoral(()->false, ()->false).withTimeout(0.1));
        NamedCommands.registerCommand("SpitCoral", intake.IntakeCoral(()->false, ()->true).withTimeout(3));

        configureBindings();
    }

    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystick.getRawAxis(1)* MaxSpeed*((0.1/joystick.getRawAxis(3)))) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(0.1/joystick.getRawAxis(3))) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(0.1/joystick.getRawAxis(3))) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.button(8).toggleOnTrue(drivetrain.applyRequest(()-> robotCentricDrive.withVelocityX(-joystick.getRawAxis(1)*MaxSpeed*((0.2/joystick.getRawAxis(3)))) // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getRawAxis(0)*MaxSpeed*((0.2/joystick.getRawAxis(3)))) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRawAxis(4)*MaxAngularRate*((0.2/joystick.getRawAxis(3))))) // Drive counterclockwise with negative X (left)
); 

        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.button(6).whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.button(3).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getRawAxis(1), -joystick.getRawAxis(0)))
            ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.button(7).and(joystick.button(4)).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.button(7).and(joystick.button(1)).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.button(8).and(joystick.button(4)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.button(8).and(joystick.button(1)).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on button B
        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        
        

        intake.setDefaultCommand(intake.IntakeCoral(()-> buttonbord.button(2).getAsBoolean(), ()-> buttonbord.button(5).getAsBoolean()));
        buttonbord.button(2).onTrue(((intake.IntakeSolenoid(()-> buttonbord.button(1).getAsBoolean()))));
       
       // wrist.setDefaultCommand(wrist.ManualWrist( ()->buttonbord.getRawAxis(0)));
        wrist.setDefaultCommand(wrist.WristPIDCommandDefault());
        buttonbord.axisGreaterThan(0, 0.1).or(buttonbord.axisLessThan(0,-0.1)).whileTrue(wrist.ManualWrist(()->buttonbord.getRawAxis(0)));
        isEnabled.whileTrue(wrist.startWristCommand());
       
       
        // elevator.setDefaultCommand((elevator.ManualElevator(()->buttonbord.getRawAxis(1), ()->wristLimiter.getAsBoolean())));
        elevator.setDefaultCommand((elevator.elevatorPIDCommandDefault(()->wristLimiter.getAsBoolean())));
       (wristLimiter).and((buttonbord.axisGreaterThan(1, 0.1).or(buttonbord.axisLessThan(1,-0.1))).whileTrue(elevator.ManualElevator(()->buttonbord.getRawAxis(1), ()->wristLimiter.getAsBoolean())));
        isEnabled.whileTrue(elevator.startCommand(wristLimiter));
       //(wristLimiter).and(buttonbord.button(3)).onTrue(elevator.topPos());
       buttonbord.button(8).onTrue(Commands.sequence(wrist.WristSafety(),wrist.WristL4())); 

       /*buttonbord.button(1).onTrue(elevator.L4Elevator(wristLimiter));
       buttonbord.button(4).onTrue(elevator.L3Elevator(wristLimiter));
       buttonbord.button(7).onTrue(elevator.L2Elevator(wristLimiter));
       buttonbord.button(10).onTrue(elevator.Home(wristLimiter));*/
      
       buttonbord.button(3).onTrue(wrist.L4Wrist());
       buttonbord.button(6).onTrue(wrist.L3Wrist());
       buttonbord.button(9).onTrue(wrist.L2Wrist());
       buttonbord.button(11).onTrue(wrist.L1Wrist());
       buttonbord.button(12).onTrue(wrist.Home());
      

     }
           
     
        
        
        
            public Command getAutonomousCommand() {
       /* Run the path selected from the auto chooser */
       return autoChooser.getSelected();
    }

 

  





}



//Sprint 4 beta code// TODO Sprint 4 beta
/* this.driveThrottleL2.and(this.driveThrottleL3.negate()).and(this.driveThrottleL4.negate()).onTrue(
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystick.getRawAxis(1)* MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.75) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.75) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(1-joystick.getRawAxis(3)*0.85)*0.75) // Drive counterclockwise with negative X (left)
    ));

    this.driveThrottleL2.and(this.driveThrottleL3).and(this.driveThrottleL4.negate()).onTrue(
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystick.getRawAxis(1)* MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.50) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.50) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(1-joystick.getRawAxis(3)*0.85)*0.50) // Drive counterclockwise with negative X (left)
    ));

    this.driveThrottleL2.and(this.driveThrottleL3).and(this.driveThrottleL4).onTrue(
            drivetrain.applyRequest(() ->
                fieldCentricDrive.withVelocityX(-joystick.getRawAxis(1)* MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.25) // Drive forward with negative Y (forward)
                .withVelocityY(-joystick.getRawAxis(0) * MaxSpeed*(1-joystick.getRawAxis(3)*0.85)*0.25) // Drive left with negative X (left)
                .withRotationalRate(-joystick.getRawAxis(4) * MaxAngularRate*(1-joystick.getRawAxis(3)*0.85)*0.25) // Drive counterclockwise with negative X (left)
    )); 
    
    driverstation.isAutonomous()
    */