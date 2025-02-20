package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightResults;

public class LimelightSubsystem extends SubsystemBase {
    private final String limelight;
    private Pose3d tagPoseRobot;
     private ShuffleboardTab dS_LimShuffleboardTab = Shuffleboard.getTab("Limelight");
         // Add the limelight to the Shuffleboard
    private GenericEntry DS_Camera;
    private GenericEntry DS_Pipeline;
    private GenericEntry DS_Tx;
    private GenericEntry DS_Ty;
    private GenericEntry DS_Confidence;
    private GenericEntry DS_TargetPoseRobotX;
    private GenericEntry DS_TargetPoseRobotY;
    private GenericEntry DS_TargetPoseRobotZ;
    private GenericEntry DS_TargetPoseRobotYaw;
    
        
        public LimelightSubsystem(String LLname) {
                limelight = LLname;
                DS_Camera =dS_LimShuffleboardTab.add("Limelight", limelight).getEntry();
                DS_Camera.setString(limelight);
                DS_Pipeline =dS_LimShuffleboardTab.add(limelight+"Pipeline", 0).getEntry();
                DS_Tx =dS_LimShuffleboardTab.add(limelight+"Tx", 0).getEntry();
                DS_Ty =dS_LimShuffleboardTab.add(limelight+"Ty", 0).getEntry();
                DS_Confidence =dS_LimShuffleboardTab.add(limelight+"Confidence", 0).getEntry();
                DS_TargetPoseRobotX =dS_LimShuffleboardTab.add(limelight+"TagToRobotX", 0).getEntry();
                DS_TargetPoseRobotY =dS_LimShuffleboardTab.add(limelight+"TagToRobotY", 0).getEntry();
                DS_TargetPoseRobotZ =dS_LimShuffleboardTab.add(limelight+"TagToRobotZ", 0).getEntry();
                DS_TargetPoseRobotYaw =dS_LimShuffleboardTab.add(limelight+"TagToRobotYaw", 0).getEntry();

                


            
        }
    
        public LimelightHelpers.PoseEstimate getVisionPose() {
            LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);
    
    
            return botPose;
        }
        public void setCameraPose_RobotSpace(double x, double y, double z, double yaw, double pitch, double roll) {
            LimelightHelpers.setCameraPose_RobotSpace(limelight, x, y, z, roll, pitch, yaw);
        }
    
        public double getTx() {
            return LimelightHelpers.getTX(limelight);
        }
        public double getTy() {
            return LimelightHelpers.getTY(limelight);
        }
    
        public void setPipeline(int pipeline) {
            LimelightHelpers.setPipelineIndex(limelight, pipeline);
        }
    
        public void setMainPIP(int mode) {
            LimelightHelpers.setStreamMode_PiPMain(limelight);
        }
    
        public void setSecondaryPIP(int mode) {
            LimelightHelpers.setStreamMode_PiPSecondary(limelight);
        }
    
    
        public double getVisionConfidence() {
            double ta = LimelightHelpers.getTA(limelight);
            boolean tv = LimelightHelpers.getTV(limelight);
            double tl = getVisionPose().latency;
    
            if (!tv) {
                return 0.0; // No tags detected
            }
    
            double confidence = Math.min(ta / 5.0, 1.0);
            if (tl > 50) {
                confidence *= 0.5; // Reduce confidence if latency is high
            }
    
            return confidence;
        }
    
        public void UpdateRobotPose_TargetSpace()
        {
            Pose3d tagPoseRobot = new Pose3d();
            LimelightResults results = LimelightHelpers.getLatestResults(limelight);
            if(results.targets_Fiducials.length > 0) {
                LimelightTarget_Fiducial tag = results.targets_Fiducials[0];
                double id = tag.fiducialID;
                tagPoseRobot = tag.getTargetPose_RobotSpace();
            }
            
            this.tagPoseRobot=tagPoseRobot;
    }
        public double getTagToRobotX() {
            return tagPoseRobot.getTranslation().getX();
        }

        public double getTagToRobotY() {
            return tagPoseRobot.getTranslation().getY();
        }


        public double getTagToRobotZ() {
            return tagPoseRobot.getTranslation().getZ();
        }

        public double getTagToRobotYaw() {
            return tagPoseRobot.getRotation().getAngle();
        }

        public Pose3d getTagPoseRobot() {
            return tagPoseRobot;
        }
 
        @Override
        public void periodic() {
      
            DS_Pipeline.setDouble(getTx());
            DS_Tx.setDouble(getTx());
            DS_Ty.setDouble(getTy());
            DS_Confidence.setDouble(getVisionConfidence());
            DS_TargetPoseRobotX.setDouble(getTagToRobotX());
            DS_TargetPoseRobotY.setDouble(getTagToRobotY());
            DS_TargetPoseRobotZ.setDouble(getTagToRobotZ());
            DS_TargetPoseRobotYaw.setDouble(getTagToRobotYaw());
        }

 
}
