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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Intake. */
  
  //CTRE//
  private final TalonFX wristTalon = new TalonFX(Constants.WristConstants.WristMotor_ID);
  public final CANcoder wristCanCoder = new CANcoder(Constants.WristConstants.WristCANcoder_ID); 
  
  //DIOs//
  /*private DigitalInput limitHI= new DigitalInput(Constants.WristConstants.HILimitPin);
  private DigitalInput limitLO= new DigitalInput(Constants.WristConstants.LOLimitPin);*/

 //Shuffle Board//
  private final PIDController wristController = new PIDController(3, 0, 0);
 private ShuffleboardTab DS_WristTab= Shuffleboard.getTab("Wrist");
 private GenericEntry DS_WristPosition=DS_WristTab.add("WristValue",0).getEntry();
  private GenericEntry DS_WristSpeed= DS_WristTab.add("Max Speed", .3).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
  
  double wristSpeed = DS_WristSpeed.getDouble(0.4); 
  
  public Wrist() {
    
   
    var WristTalonConfig = wristTalon.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 120;
    limitConfigs.StatorCurrentLimitEnable = true;
    WristTalonConfig.apply(limitConfigs);
    wristController.setSetpoint(this.getWristPosition());
    wristController.setTolerance(.01);
  } 





public static void setWristPower(TalonFX motor, double power)
  {
  motor.set(power);
  }
        

public void setWristMotor (double power){
  setWristPower(wristTalon, wristLimit(-power));
}

public void stopWristMotor(){
  this.setWristMotor(0);
}  


//TODO All power must be run through this method to prevent breaking of robot//
//Limiting method to determine power//

private double wristLimit(double power){
  double output=0;
  if (( wristCanCoder.getAbsolutePosition().getValueAsDouble()>0.42&& power<0)||(wristCanCoder.getAbsolutePosition().getValueAsDouble()<0.01&& power>0))
    {
      output =0;
    }
  else
    {
      output=power;
    }
  return output;
    } 


    public Trigger wristLimiter()
    {
      return new Trigger(()-> this.getWristPosition()<=.09);
       
    }
  

    
  
 
 //PID methods//
  public double getWristPosition(){
    return (wristCanCoder.getAbsolutePosition().getValueAsDouble())/* TODO Multiple to make read in degrees */; 
  }

  public void setWristPID (double setPoint){
    this.wristController.setSetpoint(setPoint);
  }
  
  public void executeWristPID(){
    setWristMotor((this.wristController.calculate(this.getWristPosition()))); 
  }

  public boolean wristAtSetpoint(){
   return this.wristController.atSetpoint();
  } 
  //PID methods end// 

  
//Commands// 
public Command WristPIDCommandDefault()
{
  return new FunctionalCommand(
    ()->{},


    ()-> this.executeWristPID(),

    interrupted->{},
    
    ()->{return false;},
     this);
} 

  public Command ManualWrist(DoubleSupplier wristJoystick ){
    return new FunctionalCommand(
      ()-> {}, 
      
      ()-> this.setWristMotor(wristJoystick.getAsDouble()*0.2), 
    
      interrupted -> this.setWristPID(getWristPosition()),
    
      
      ()-> {return false;}, 
    
      this
    
    );


  }

  public Command startWristCommand(){
    

  return new FunctionalCommand(
    ()->{},
    
    ()-> {} ,
     
    interrupted-> 
    
    this.setWristPID(this.getWristPosition()), 
     
    ()->{return false;}, 
     
    this);
}



public Command L4Wrist(){
  return runOnce(()->{this.setWristPID(.211);
  }); 

}  


public Command L3Wrist(){
  return runOnce(()->{this.setWristPID(.3288);
  }); 

}   

public Command L2Wrist(){
  return runOnce(()->{this.setWristPID(.3413);
  }); 

} 

public Command L1Wrist(){
  return runOnce(()->{this.setWristPID(.445);
  }); 

}   

public Command Home(){
  return runOnce(()->this.setWristPID(.02)
  ); 

}  

public Command WristSafety()
{
  return new FunctionalCommand(
    ()->{ this.setWristPID(.045);},

    ()-> {
           
           this.executeWristPID();
           
         },

    interrupted->{},
    
    ()-> this.wristAtSetpoint(),
    
    this);
} 

public Command WristL4()
{
  return new FunctionalCommand(
    ()->{ this.setWristPID(.211);},

    ()-> {
           
           this.executeWristPID();
           
         },

    interrupted->{},
    
    ()-> this.wristAtSetpoint(),
    
    this);
}   

  
//Command end//


  @Override
  public void periodic() {
    
      this.DS_WristPosition.setDouble(getWristPosition());
      this.wristSpeed = this.DS_WristSpeed.getDouble(0.2);
      
    // This method will be called once per scheduler run
  }
}
