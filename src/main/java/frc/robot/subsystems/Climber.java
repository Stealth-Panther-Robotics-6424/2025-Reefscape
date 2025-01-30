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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX climberTalon = new TalonFX(Constants.ClimberConstants.ClimberMotor_ID);
  public final CANcoder climberCanCoder = new CANcoder(Constants.ClimberConstants.ClimberCANcoder_ID); 
  


 //Shuffle Board//
  private ShuffleboardTab DS_ClimberTab= Shuffleboard.getTab("Climber");
  private GenericEntry DS_ClimberPosition= DS_ClimberTab.add("ClimberValue",0).getEntry();
  private GenericEntry DS_maxElevatorSpeed = DS_ClimberTab.add("Climber Speed", .2).getEntry();
  private GenericEntry DS_forwardLimit= DS_ClimberTab.add("Forward Limit", true).getEntry();
  private GenericEntry DS_reverseLimit= DS_ClimberTab.add("Reverse Limit", true).getEntry();
  //double maxClimber = this.DS_maxClimberSpeed.getDouble(0.2);
  
  
  
  public Climber() {
    climberTalon.setNeutralMode(NeutralModeValue.Brake);
   

    var ClimberConfig = climberTalon.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    ClimberConfig.apply(limitConfigs);
   }

   private void setMotorPower(TalonFX motor, double power){
    motor.set(power);
  }

  /*public void setClimberMotor (double power)  
    {
    setMotorPower(climberTalon, climberLimit(power) );
    
    }

  /* private double climberLimit(double power){
      double output=0;
    if ((climberCanCoder.getAbsolutePosition().getValueAsDouble()>0.4&& power<0)||(climberCanCoder.getAbsolutePosition().getValueAsDouble()<0.01&& power>0))
      {  
        output = 0;
      }
    else{
      output=power;
      }
    return output;
    } 

  public double getClimberPosition(){
      return (climberCanCoder.getAbsolutePosition().getValueAsDouble())TODO Multiple to make read in degrees ; 
    } 

  //Commands
  
    public Command ManualWrist(DoubleSupplier wristJoystick ){
    return new FunctionalCommand(
      ()-> {}, 
      
      ()-> this.setWristMotor(wristJoystick.getAsDouble()*0.2), 
    
      interrupted -> this.setWristPID(getWristPosition()),
    
      
      ()-> {return false;}, 
    
      this
    
    );


  } */






  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
