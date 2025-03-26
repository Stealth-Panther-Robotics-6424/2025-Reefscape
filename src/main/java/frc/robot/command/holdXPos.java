// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.command;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class holdXPos extends Command {
 
  double currentX ;
  CommandSwerveDrivetrain drivetrain;
  double HoldXPos;
  DoubleSupplier ySupplier;
  DoubleSupplier rotSupplier;
  double MaxSpeed;
  DoubleSupplier throttle;
  DoubleSupplier elevatorThrottle;
  SwerveRequest swerveRequest;
  PIDController pidController = new PIDController(0.1, 0, 0);
  double xControlOutput;
  private final SwerveRequest.FieldCentric fieldCentricDrive ;
  /** Creates a new holdXPos. */
  public holdXPos(CommandSwerveDrivetrain drivetrain,
                  double HoldXPos,
                  double MaxSpeed,
                  double MaxAngularRate ,
                  DoubleSupplier ySupplier,
                  DoubleSupplier rotSupplier,
                  DoubleSupplier throttle,
                  DoubleSupplier elevatorThrottle) 
  {
    this.drivetrain = drivetrain;
    this.HoldXPos = HoldXPos;
    this.ySupplier = ySupplier;
    this.rotSupplier = rotSupplier;
   
    this.MaxSpeed = MaxSpeed;
    this.throttle = throttle;
    this.elevatorThrottle = elevatorThrottle;
    addRequirements(drivetrain);
    fieldCentricDrive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 

    currentX = drivetrain.getState().Pose.getX();
    xControlOutput = pidController.calculate(currentX, HoldXPos);

    drivetrain.setControl( fieldCentricDrive.withVelocityX(xControlOutput * MaxSpeed * throttle.getAsDouble()
                                                           *elevatorThrottle.getAsDouble())
                                            .withVelocityY(-ySupplier.getAsDouble() * MaxSpeed * throttle.getAsDouble()
                                                           *elevatorThrottle.getAsDouble())                                                                                        
                                            .withRotationalRate(-rotSupplier.getAsDouble() * MaxSpeed * throttle.getAsDouble()
                                                                *elevatorThrottle.getAsDouble()));

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xControlOutput = pidController.calculate(currentX, HoldXPos);

    drivetrain.setControl( fieldCentricDrive.withVelocityX(xControlOutput * MaxSpeed * throttle.getAsDouble()
                                                           *elevatorThrottle.getAsDouble())
                                            .withVelocityY(-ySupplier.getAsDouble() * MaxSpeed * throttle.getAsDouble()
                                                           *elevatorThrottle.getAsDouble())                                                                                        
                                            .withRotationalRate(-rotSupplier.getAsDouble() * MaxSpeed * throttle.getAsDouble()
                                                                *elevatorThrottle.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl( fieldCentricDrive.withVelocityX(0)
                                            .withVelocityY(0)                                                                                        
                                            .withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
