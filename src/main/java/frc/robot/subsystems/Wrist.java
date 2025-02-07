// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /**
   * Creates a new Wrist subsystem. This class controls the wrist motor and
   * allows for various actions like manual control, setting wrist positions,
   * and running PID-based control to maintain precise wrist positioning.
   */

  // CTRE hardware objects for controlling the wrist motor and reading wrist
  // position
  private final TalonFX wristTalon = new TalonFX(Constants.WristConstants.WristMotor_ID);
  // A TalonFX motor controller for the wrist motor, used to control the wrist's
  // movement based on power input
  public final CANcoder wristCanCoder = new CANcoder(Constants.WristConstants.WristCANcoder_ID);
  // A CANcoder sensor that provides the current position of the wrist, which is
  // crucial for feedback control

  // DIOs for limit switches are commented out but could be used to limit wrist
  // movement for safety
  /*
   * private DigitalInput limitHI = new
   * DigitalInput(Constants.WristConstants.HILimitPin);
   * High limit switch to prevent the wrist from moving too far in the upward
   * direction.
   * private DigitalInput limitLO = new
   * DigitalInput(Constants.WristConstants.LOLimitPin);
   * Low limit switch to prevent the wrist from moving too far in the downward
   * direction.
   */

  // Shuffleboard setup to display wrist data for debugging and tuning
  private final PIDController wristController = new PIDController(3, 0, 0);
  // PID controller to maintain wrist position by calculating the appropriate
  // motor output
  private ShuffleboardTab DS_WristTab = Shuffleboard.getTab("Wrist");
  // Create a new tab on the Shuffleboard for wrist data visualization
  private GenericEntry DS_WristPosition = DS_WristTab.add("WristValue", 0).getEntry();
  // Add an entry to display the wrist's current position on Shuffleboard
  private GenericEntry DS_WristSpeed = DS_WristTab.add("Wrist Speed", .3).withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  // Add a speed slider to adjust the maximum wrist speed in real-time through
  // Shuffleboard

  double wristSpeed = DS_WristSpeed.getDouble(0.4);
  boolean wristSafe = false;
  boolean safeFold = false;
  // Retrieve the wrist speed from the Shuffleboard, defaulting to 0.4 if not set

  public Wrist() {
    // Wrist constructor to configure motor controller settings and PID controller

    // Configure motor controller current limits to prevent overcurrent damage
    var WristTalonConfig = wristTalon.getConfigurator();
    // Get the configuration interface for the TalonFX motor controller
    var limitConfigs = new CurrentLimitsConfigs();
    // Create a new configuration object for current limits
    limitConfigs.StatorCurrentLimit = 120;
    // Set the stator current limit to 120 Amps to protect the motor from
    // overcurrent
    limitConfigs.StatorCurrentLimitEnable = true;
    // Enable the stator current limit to prevent motor damage
    WristTalonConfig.apply(limitConfigs);
    // Apply the current limit configuration to the motor controller

    wristController.setSetpoint(this.getWristPosition());
    // Set the initial target position for the PID controller based on the current
    // wrist position
    wristController.setTolerance(.007);
    // Set the tolerance for the PID controller (how close the motor has to be to
    // the setpoint to stop adjusting)
  }

  // Method to set power to the wrist motor directly
  public static void setWristPower(TalonFX motor, double power) {
    motor.set(power);
    // Set the motor power using the TalonFX motor controller
  }

  // Method to set wrist motor power, limiting the power to prevent over-travel
  public void setWristMotor(double power) {
    setWristPower(wristTalon, wristLimit(-power));
    // Apply the wrist power limit to ensure the wrist doesn't exceed its mechanical
    // limits
  }

  // Method to stop the wrist motor by setting the power to zero
  public void stopWristMotor() {
    this.setWristMotor(0);
    // Set the motor power to zero to stop the wrist from moving
  }

  // Method to limit wrist motor power based on the current wrist position to
  // avoid damaging the wrist
  private double wristLimit(double power) {
    double output = 0;
    // Check if wrist movement would exceed the physical limits based on wrist
    // position
    if ((wristCanCoder.getAbsolutePosition().getValueAsDouble() > 0.446 && power < 0) ||
        (wristCanCoder.getAbsolutePosition().getValueAsDouble() < 0.038 && power > 0)
        || (!this.safeFold && power < 0)) {
      output = 0;
      // If the wrist is at its limit, prevent further movement by setting output to
      // zero
    } else {
      output = power;
      // If within bounds, allow the power to pass through to the motor
    }
    return output;
    // Return the (possibly limited) motor power
  }

  // Trigger to limit wrist movement if the wrist is too low (below position 0.09)
  public Trigger wristLimiter() {
    return new Trigger(() -> this.getWristPosition() <= .09);
    // Create a trigger that activates when the wrist position is below 0.09
  }

  // Trigger to activate intake if the wrist is at L1 (above or equal to position
  // 0.42)
  public Trigger wristIntake() {

    return new Trigger(() -> this.getWristPosition() >= .42);
    // Create a trigger that activates when the wrist position is above or equal to
    // 0.42
  }

  // Method to retrieve the current wrist position from the CANcoder sensor
  public double getWristPosition() {
    return (wristCanCoder.getAbsolutePosition().getValueAsDouble());
    // Return the current wrist position as a double value from the CANcoder sensor
  }

  // Method to set the desired target wrist position for PID control
  public void setWristPID(double setPoint) {
    this.wristController.setSetpoint(setPoint);
    // Set the desired position (setpoint) for the wrist using the PID controller
  }

  // Method to execute PID control to adjust the motor power and move the wrist
  // towards the target position
  public void executeWristPID() {
    setWristMotor((this.wristController.calculate(this.getWristPosition())));
    // Calculate the appropriate motor power based on the PID controller and apply
    // it to the wrist motor
  }

  // Method to check if the wrist has reached its target position based on the PID
  // tolerance
  public boolean wristAtSetpoint() {
    return this.wristController.atSetpoint();
    // Return true if the wrist has reached its target position, within the
    // specified tolerance (0.01)
  }

  private boolean wristSafe() {

    if (this.getWristPosition() <= 0.045) {
      wristSafe = true;
    } else {
      wristSafe = false;
    }
    return wristSafe;
  }

  // Command to run the wrist motor with PID control
  public Command WristPIDCommandDefault(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.safeFold = canFold.getAsBoolean();
        }, // Initialize: No action needed
        () -> this.executeWristPID(), // Execute: Run the PID control to adjust wrist position
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> {
          return false; // Finish condition: Always false, meaning the command never finishes
                        // automatically
        },
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  // Command for manual control of the wrist motor, using joystick input for wrist
  // movement
  public Command ManualWrist(DoubleSupplier wristJoystick, BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.safeFold = canFold.getAsBoolean();
        }, // Initialize: No action needed
        () -> this.setWristMotor(wristJoystick.getAsDouble() * 0.3), // Execute: Set wrist motor power based
                                                                     // this.wristSpeed
                                                                     // on joystick
        // input (scaled by 0.2 for control)
        interrupted -> this.setWristPID(getWristPosition()), // Interrupted: Reset to the current wrist position if
                                                             // interrupted
        () -> {
          return false; // Finish condition: Always false, meaning the command never finishes
                        // automatically
        },
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  // Command to start the wrist motor but with no action (used for state
  // transitions)
  public Command startWristCommand() {
    return this.runOnce(() -> {
      this.setWristPID(this.getWristPosition());
    });
  }

  // Command to ensure the wrist is moved to a safe position
  public Command WristSafety(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.setWristPID(.045);
          this.safeFold = canFold.getAsBoolean(); // Set wrist to a safe position of 0.045
        },
        () -> {
          this.executeWristPID(); // Execute PID control to move wrist to the safe position
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if the wrist has reached the safe position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  // Commands for moving the wrist to various predefined positions (L1, L2, L3,
  // L4)
  public Command WristL1(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.setWristPID(.459);
          this.safeFold = canFold.getAsBoolean(); // Set wrist to position L1 (0.445)
        },
        () -> {
          this.executeWristPID(); // Execute PID control to move wrist to L1
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if wrist has reached L1 position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  public Command WristL2(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.setWristPID(.3413);
          this.safeFold = canFold.getAsBoolean(); // Set wrist to position L2 (0.3413)
        },
        () -> {
          this.executeWristPID(); // Execute PID control to move wrist to L2
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if wrist has reached L2 position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  public Command WristL3(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.setWristPID(.3288);
          this.safeFold = canFold.getAsBoolean(); // Set wrist to position L3 (0.3288)
        },
        () -> {
          this.executeWristPID(); // Execute PID control to move wrist to L3
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if wrist has reached L3 position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  public Command WristL4(BooleanSupplier canFold) {
    return new FunctionalCommand(
        () -> {
          this.setWristPID(.211);
          this.safeFold = canFold.getAsBoolean(); // Set wrist to position L4 (0.211)
        },
        () -> {
          this.executeWristPID(); // Execute PID control to move wrist to L4
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if wrist has reached L4 position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  // Command to exit wrist state, keeping it in its current position
  public Command ExitState() {
    return new FunctionalCommand(
        () -> {
          this.wristController.setSetpoint(this.getWristPosition()); // Set the current wrist position as the target
                                                                     // setpoint
        },
        () -> {
          this.executeWristPID(); // Execute PID control to hold the wrist at its current position
        },
        interrupted -> {
        }, // Interrupted: No specific action when interrupted
        () -> this.wristAtSetpoint(), // Finish condition: Check if wrist is holding the current position
        this); // Subsystem: This command is bound to the Wrist subsystem
  }

  // Periodic method called once per scheduler run, used for updating real-time
  // data
  @Override
  public void periodic() {
    this.DS_WristPosition.setDouble(getWristPosition()); // Update the wrist's current position on Shuffleboard
    this.wristSpeed = this.DS_WristSpeed.getDouble(0.2); // Update wrist motor speed based on Shuffleboard slider input

    // This method runs periodically to keep the wrist data current and allow for
    // real-time adjustments
  }
}