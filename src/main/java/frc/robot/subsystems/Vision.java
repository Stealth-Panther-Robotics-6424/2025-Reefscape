// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  public final LimelightSubsystem bowLL = new LimelightSubsystem("bowlime");
  public final LimelightSubsystem aftLL = new LimelightSubsystem("aftlime");

  /** Creates a new Vision. */
  public Vision() {

    LimelightHelpers.setCameraPose_RobotSpace("bowlime", 0.3210213798,
        0.2041934194, 0.2244396832, 0.618624, 15.61692, 3.312392);
    LimelightHelpers.setCameraPose_RobotSpace("aftlime", 0.1171411424, 0, 0.9671403314, 0, 22, 180);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void updatePoseEstimate(CommandSwerveDrivetrain drivetrain) {
    double[] aftpose = NetworkTableInstance.getDefault().getTable("aftlime").getEntry("botpose_wpiblue")
        .getDoubleArray(new double[6]);
    LimelightHelpers.PoseEstimate bowpose = bowLL.getVisionPose();
    if (aftpose[3] > 0) {
      SmartDashboard.putBoolean("else", false);
      SmartDashboard.putNumber("aftPosX", aftpose[0]);

      if (aftpose[0] < 0.7) {
        {

          // drivetrain.addVisionMeasurement(aftpose.pose, aftpose.timestampSeconds,
          // VecBuilder.fill(0.1, 0.1, 0.1));

        }
      } else {
        SmartDashboard.putBoolean("else", true);
      }
    }
    if (bowpose != null) {
      if (bowpose.rawFiducials[0].ambiguity < 0.7) {
        drivetrain.addVisionMeasurement(bowpose.pose, bowpose.timestampSeconds, VecBuilder.fill(0.1, 0.1, 0.1));
      }
    }

  }
}