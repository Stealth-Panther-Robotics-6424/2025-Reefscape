// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX elevatorTalonPort = new TalonFX(Constants.ElevatorConstants.ElevatorPortMotor_ID);
  private final TalonFX elevatorTalonStrb = new TalonFX(Constants.ElevatorConstants.ElevatorStrbMotor_ID);
  private boolean canLift = false;

  private final PIDController elevatorController = new PIDController(0.028, 0, 0); // OG value 0.0002348

  // Shuffle Board//
  private ShuffleboardTab DS_ElevatorTab = Shuffleboard.getTab("Elevator");
  private GenericEntry DS_ElevatorPosition = DS_ElevatorTab.add("ElevatorValue", 0).getEntry();
  private GenericEntry DS_maxElevatorSpeed = DS_ElevatorTab.add("Elevator Speed", .2).getEntry();
  private GenericEntry DS_forwardLimit = DS_ElevatorTab.add("Forward Limit", true).getEntry();
  private GenericEntry DS_reverseLimit = DS_ElevatorTab.add("Reverse Limit", true).getEntry();
  double maxElevator = this.DS_maxElevatorSpeed.getDouble(0.2);

  public Elevator() {

    elevatorTalonPort.setNeutralMode(NeutralModeValue.Brake);
    elevatorTalonStrb.setNeutralMode(NeutralModeValue.Brake);

    var ElevatorPortConfig = elevatorTalonPort.getConfigurator();
    var ElevatorStrbConfig = elevatorTalonStrb.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    ElevatorPortConfig.apply(limitConfigs);
    ElevatorStrbConfig.apply(limitConfigs);

    elevatorController.setSetpoint(getElevatorPosition());
    elevatorController.setTolerance(1);
    // elevatorTalonStrb.setPosition(0);
  }

  public void teleopInit() {
    elevatorController.setSetpoint(getElevatorPosition());
  }

  private void setMotorPower(TalonFX motor, double power) {
    motor.set(power);
  }

  public void setElevatorMotor(double power) {
    setMotorPower(elevatorTalonPort, elevatorLimit(-power));
    setMotorPower(elevatorTalonStrb, elevatorLimit(power));
  }

  private static boolean isEnabled() {
    return DriverStation.isEnabled();
  }

  private double elevatorLimit(double power) {
    double output = 0;
    if (!canLift || (elevatorTalonStrb.getPosition().getValueAsDouble() >= 115 && power < 0)
        || (elevatorTalonStrb.getPosition().getValueAsDouble() <= 0.5 && power > 0)) {
      output = .02;
    } else {
      output = power;
    }
    return output;
  }

  // PID methods

  public void setElevatorPID(double setPoint) {
    if (canLift) {
      this.elevatorController.setSetpoint(setPoint);
    } else {
      this.elevatorController.setSetpoint(this.getElevatorPosition());
    }
  }

  public double getElevatorPosition() {
    return (elevatorTalonStrb.getPosition().getValueAsDouble())/* TODO Multiple to make read in inches */;
  }

  public void executeElevatorPID() {
    setElevatorMotor((this.elevatorController.calculate(this.getElevatorPosition())));
  }

  public boolean elevatorAtSetpoint() {
    return this.elevatorController.atSetpoint();
  }

  // PID methods end//

  // Commands//

  public Command ManualElevator(DoubleSupplier elevatorJoystick, BooleanSupplier wristLimiter) {

    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean();
        },

        () -> {
          this.setElevatorMotor(elevatorJoystick.getAsDouble() * .2);
          this.canLift = wristLimiter.getAsBoolean();
        },

        interrupted -> this.setElevatorPID(this.getElevatorPosition()),

        () -> {
          return false;
        },

        this);
  }

  public Command startCommand(BooleanSupplier wristLimiter) {

    return new FunctionalCommand(
        () -> {
        },

        () -> {
        },

        interrupted -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.setElevatorPID(this.getElevatorPosition());
        },

        () -> {
          return false;
        },

        this);
  }

  public Command elevatorPIDCommandDefault(BooleanSupplier wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean();
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID();
        },

        interrupted -> {
          this.canLift = wristLimiter.getAsBoolean();
        },

        () -> {
          return false;
        },

        this);
  }

  public Command Home(BooleanSupplier wristLimiter) {
    return runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean();
      this.setElevatorPID(0);
    });

  }

  public Command L2Elevator(BooleanSupplier wristLimiter) {
    return runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean();
      this.setElevatorPID(19);
    });

  }

  public Command L3Elevator(BooleanSupplier wristLimiter) {
    return runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean();
      this.setElevatorPID(52.7);
    });

  }

  public Command L4Elevator(BooleanSupplier wristLimiter) {
    return runOnce(() -> {
      this.canLift = wristLimiter.getAsBoolean();
      this.setElevatorPID(113);
    });

  }

  public Command elevatorL4(BooleanSupplier wristLimiter) {
    return new FunctionalCommand(
        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.setElevatorPID(113);
        },

        () -> {
          this.canLift = wristLimiter.getAsBoolean();
          this.executeElevatorPID();

        },

        interrupted -> {
        },

        () -> this.elevatorAtSetpoint(),

        this);
  }

  // Commands End//

  @Override
  public void periodic() {

    this.maxElevator = this.DS_maxElevatorSpeed.getDouble(0.2);
    this.DS_ElevatorPosition.setDouble(getElevatorPosition());
    this.DS_forwardLimit.setDouble((this.elevatorTalonStrb.getForwardLimit().getValueAsDouble()));
    this.DS_reverseLimit.setDouble(this.elevatorTalonStrb.getReverseLimit().getValueAsDouble());
    // This method will be called once per scheduler run
  }

}

// Sprint 4.0 Beta Code TODO Sprint 4 beta//
/*
 * public Trigger driveThrottleL2(){
 * return new Trigger(()-> this.getElevatorPosition()>=115);
 * 
 * }
 * 
 * 
 * public Trigger driveThrottleL3(){
 * return new Trigger(()-> this.getElevatorPosition()>=115);
 * 
 * }
 * 
 * 
 * public Trigger driveThrottleL4(){
 * return new Trigger(()-> this.getElevatorPosition()>=115);
 * 
 * }
 */
