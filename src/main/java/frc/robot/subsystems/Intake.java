// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
static TalonFX intakeTop = new TalonFX(Constants.IntakeConstants.TopIntakeCAN_ID); 
static DigitalInput BBIntake = new DigitalInput(Constants.IntakeConstants.BeamBreakPinIntake);
static DigitalInput BBDis = new DigitalInput(Constants.IntakeConstants.BeamBreakPinDis);
static DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);

private ShuffleboardTab DS_IntakeTab= Shuffleboard.getTab("Intake");
private GenericEntry DS_intakeBeamBreak= DS_IntakeTab.add("BB Forward", true).getEntry();
private GenericEntry DS_disBeamBrake= DS_IntakeTab.add("BB Back", true).getEntry();

//private DigitalInput beamBreak= new DigitalInput(Constants.IntakeConstants.BeamBreakPin);

 

  public Intake() {
    var intakeTopConfig = intakeTop.getConfigurator();
    
    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 60;
    limitConfigs.StatorCurrentLimitEnable = true;

    intakeTopConfig.apply(limitConfigs);
    intakeSolenoid.set(Value.kReverse);

  }

  

  public static void setIntakePower(TalonFX motor, double power)
  {
  motor.set(power);
  }


  public static void setIntake(double power) 
  {
 
  setIntakePower(intakeTop, -power);
  }

  private double intakeTestCode(double power){
    double output=0;
    if(!this.disBBValue()||!this.intakeBBValue()){
        if(!this.disBBValue()){
          output = 0;
        }
        else{
          output = 0.05;
        }
    }
else{
  output = power;
}
    return output;
  }
  
  public static void stopIntake() 
  {
  
  setIntakePower(intakeTop, 0);
  }

  public boolean intakeBBValue(){
    return BBIntake.get();
  }

  public boolean disBBValue(){
    return BBDis.get();
  }


  public void manualIntake(BooleanSupplier InSupplier, BooleanSupplier OutSupplier ){
    if(OutSupplier.getAsBoolean()||InSupplier.getAsBoolean()){    //Intake Code
      if(OutSupplier.getAsBoolean()){
       setIntake(intakeTestCode(1));
        } 
     else{
      
        setIntake(1); }
        
      }   
      else{
        setIntake(0);
      }




  }

  
  
  public Command IntakeCoral(BooleanSupplier InSupplier, BooleanSupplier OutSupplier )
  {


    return new FunctionalCommand(
      ()-> {}, 
      
      ()-> this.manualIntake(InSupplier, OutSupplier), 
      interrupted -> stopIntake()
      ,
      
      ()-> {return false;}, 
      
      this
      
    ); 

   
    
  } 



  public Command IntakeSolenoid(BooleanSupplier IntakeSupplier)
  {
      
    return this.runOnce(() ->intakeSolenoid.toggle());    
  } 

    

  


  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    this.DS_intakeBeamBreak.setBoolean(intakeBBValue());
    this.DS_disBeamBrake.setBoolean(disBBValue());
  }
}


