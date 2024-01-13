// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionContants;

public class AprilTag extends SubsystemBase {
  /** Creates a new AprilTag. */
  PhotonCamera camera = new PhotonCamera("photonvision");
  double tx;
  double ty;
  double tz;
  public AprilTag() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  
  public void findAngleOfShootingToSpeaker()
  {
    //3D Reference point middle of speaker at bottem edge of opening
    double tx = 0;
    double ty = 0;
    double tz = 0;
    double dist = Math.sqrt(Math.pow(tx,2) + Math.pow(ty,2));
    double theta = Math.atan(ty/tx); //i forgot which way ty and tx
    double top_dist = VisionContants.SPEAKER_EXTRUSION_INCH * Math.sin(theta);
    double angle = Math.atan(VisionContants.CARPET_TO_SPEAKER_BOT_INCH / (dist - top_dist));
    double top_angle = Math.atan(VisionContants.CARPET_TO_SPEAKER_BOT_INCH / (dist));
    //Adjust Shooter vertical using values
  }
}
