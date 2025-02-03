// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  // This creates a new TalonFX motor controller that will control the climber
  // motor.
  // The TalonFX motor controller is identified by a unique motor ID from the
  // Constants file.
  private final TalonFX climberTalon = new TalonFX(Constants.ClimberConstants.ClimberMotor_ID);

  // This creates a new CANcoder, which is a sensor used to track the absolute
  // position of the climber mechanism.
  // The CANcoder is identified by a unique ID, again taken from the Constants
  // file.
  public final CANcoder climberCanCoder = new CANcoder(Constants.ClimberConstants.ClimberCANcoder_ID);

  // Define the intake tray motor, which is controlled by a VictorSP motor
  // controller.
  static VictorSP intakeTray = new VictorSP(Constants.ClimberConstants.IntakeTray_Pin);

  // Setting up the Shuffleboard tab to display data related to the climber
  // subsystem in real-time on the driver's station.
  // Shuffleboard is a graphical interface that displays various robot data for
  // debugging and monitoring.
  private ShuffleboardTab DS_ClimberTab = Shuffleboard.getTab("Climber");

  // Adding widgets to the Shuffleboard that will display specific climber data:
  // - DS_ClimberPosition: Displays the climber's position.
  // - DS_maxElevatorSpeed: Displays the maximum speed at which the climber will
  // move.
  // - DS_forwardLimit: Displays a boolean value indicating whether the climber
  // can move forward.
  // - DS_reverseLimit: Displays a boolean value indicating whether the climber
  // can move in reverse.
  private GenericEntry DS_ClimberPosition = DS_ClimberTab.add("ClimberValue", 0).getEntry();
  private GenericEntry DS_maxElevatorSpeed = DS_ClimberTab.add("Climber Speed", .2).getEntry();
  private GenericEntry DS_forwardLimit = DS_ClimberTab.add("Forward Limit", true).getEntry();
  private GenericEntry DS_reverseLimit = DS_ClimberTab.add("Reverse Limit", true).getEntry();

  // Constructor for the Climber subsystem. This is called when the subsystem is
  // instantiated.
  // This sets up configurations for the motor and other components that are part
  // of the climber.
  public Climber() {

    // Setting the TalonFX motor controller to "Brake" mode.
    climberTalon.setNeutralMode(NeutralModeValue.Brake);

    // Here we get a configurator for the climber motor, which allows us to
    // configure various settings related to the motor.
    var ClimberConfig = climberTalon.getConfigurator();

    // Creating an object to store current limit configurations for the motor.
    var limitConfigs = new CurrentLimitsConfigs();

    // Setting the stator current limit to 60 Amps. This prevents the motor from
    // drawing more current than this value
    limitConfigs.StatorCurrentLimit = 60;

    // Enabling the stator current limit to make sure it takes effect.
    limitConfigs.StatorCurrentLimitEnable = true;

    // Applying the configured current limits to the motor using the ClimberConfig
    // object.
    ClimberConfig.apply(limitConfigs);
  }

  // Helper method to set the motor power. It directly controls the TalonFX motor
  // by applying the provided power.
  private void setMotorPower(TalonFX motor, double power) {
    motor.set(power); // Set the motor's power to the specified value.
  }

  // Method to set the power of the intake tray motor (VictorSP). The power is
  // passed as a parameter.
  public static void setTrayPower(VictorSP victor, double power) {
    victor.set(power); // Set the tray motor to the desired power.
  }

  // Method to control the tray motor with a specific power. The power is passed
  // as a parameter.
  public static void setTray(double power) {
    setTrayPower(intakeTray, power); // Set the tray motor to the specified power.
  }

  /*
   * The following commented-out method was likely intended to control the climber
   * motor with position limits.
   * Based on the position from the CANcoder, the power sent to the motor would be
   * adjusted to prevent the climber
   * from moving too far in certain directions (e.g., when it's already at the top
   * or bottom).
   * It’s currently commented out, and could be useful if the limiting
   * functionality is needed.
   */

  /*
   * public void setClimberMotor (double power)
   * {
   * // Calls the setMotorPower method with the power returned from the
   * climberLimit method
   * setMotorPower(climberTalon, climberLimit(power) );
   * 
   * }
   * 
   * /* private double climberLimit(double power){
   * // Initially sets the output power to 0
   * double output=0;
   * 
   * // This condition checks if the climber has reached its upper limit (position
   * > 0.4) and the requested power is negative,
   * // OR if the climber has reached its lower limit (position < 0.01) and the
   * requested power is positive.
   * // If either of these conditions is true, the motor power is set to 0 to
   * prevent movement beyond limits.
   * if ((climberCanCoder.getAbsolutePosition().getValueAsDouble()>0.4&&
   * power<0)||(climberCanCoder.getAbsolutePosition().getValueAsDouble()<0.01&&
   * power>0))
   * {
   * output = 0; // Prevent movement beyond the limits of the climber mechanism
   * }
   * else{
   * output=power; // Allow the requested power if within the safe position limits
   * }
   * return output; // Return the constrained power value
   * }
   * 
   * // This method returns the current position of the climber from the CANcoder
   * sensor.
   * // The TODO suggests converting the raw position data to degrees (likely a
   * scaling factor is needed).
   * public double getClimberPosition(){
   * return (climberCanCoder.getAbsolutePosition().getValueAsDouble())TODO
   * Multiple to make read in degrees ;
   * }
   * 
   * // The following commented-out method defines a command for manually
   * controlling the wrist of the climber system
   * // based on joystick input. The code is currently not active, but might be
   * implemented later.
   * // The joystick input would adjust the wrist motor’s position with a scaling
   * factor (0.2 in this case).
   * // The method uses a FunctionalCommand to execute the manual wrist control.
   * 
   * //Commands
   * 
   * public Command ManualWrist(DoubleSupplier wristJoystick ){
   * return new FunctionalCommand(
   * ()-> {}, // Initialize the command (no setup required)
   * 
   * ()-> this.setWristMotor(wristJoystick.getAsDouble()*0.2), // During the
   * command execution, set the wrist motor's power based on joystick input,
   * scaled by 0.2
   * 
   * // If the command is interrupted, reset the wrist position using PID control
   * to return to a target position.
   * interrupted -> this.setWristPID(getWristPosition()),
   * 
   * // End condition for the command (this version of the command does not have
   * an end condition, so it will never automatically finish)
   * ()-> {return false;},
   * 
   * // The subsystem (Climber) that this command is associated with. It will be
   * run on the Climber subsystem.
   * this
   * 
   * );
   * }
   */

  // This method is called periodically by the robot’s scheduler to update the
  // subsystem.
  // It’s typically used to update variables or perform tasks that need to run
  // repeatedly.

  // Command to manually control the intake tray motor.
  // This command allows for manual control of the tray, moving it forward or
  // backward.
  public Command TrayManual() {
    return new FunctionalCommand(() -> {
    }, () -> {
      this.setTray(1); // Set the tray motor to forward power.
    }, interrpted -> {
      this.setTray(-1); // Set the tray motor to reverse power if the command is interrupted.
    }, () -> {
      return false; // Command should run indefinitely until interrupted or canceled.
    }, this); // Pass this subsystem (Intake) to the command.
  }

  @Override
  public void periodic() {
    // Currently, the periodic method is empty. You could add periodic tasks such as
    // updating values or performing checks here.
  }
}