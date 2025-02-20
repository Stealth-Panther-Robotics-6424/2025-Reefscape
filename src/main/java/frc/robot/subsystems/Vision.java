// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private final LimelightSubsystem bowLL = new LimelightSubsystem("bowLimelight");
  private final LimelightSubsystem aftLL= new LimelightSubsystem("aftLimelight");
  
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void updatePoseEstimate(CommandSwerveDrivetrain drivetrain){
    LimelightHelpers.PoseEstimate aftpose = aftLL.getVisionPose();
    LimelightHelpers.PoseEstimate bowpose = bowLL.getVisionPose();
    if (aftpose.rawFiducials[0].ambiguity<0.7&&aftpose.rawFiducials.length>0){ {
      
      drivetrain.addVisionMeasurement(aftpose.pose,aftpose.timestampSeconds);
    }
    if (bowpose.rawFiducials[0].ambiguity<0.7&&bowpose.rawFiducials.length>0){
      drivetrain.addVisionMeasurement(bowpose.pose,bowpose.timestampSeconds);
    }
    
  }



  
}
}