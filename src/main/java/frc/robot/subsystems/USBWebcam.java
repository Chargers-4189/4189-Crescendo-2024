// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets; 
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class USBWebcam extends SubsystemBase {
  /** Creates a new Camera. */
  private UsbCamera camera = CameraServer.startAutomaticCapture("Camera", CameraConstants.kUSBPort);
  private final ShuffleboardTab tab = Shuffleboard.getTab(CameraConstants.kShuffleboardTitle);
  
  public USBWebcam() {
    tab.add("camera",camera)
    .withPosition(CameraConstants.kPosY, CameraConstants.kPosY)
    .withWidget(BuiltInWidgets.kCameraStream);
    camera.setResolution(CameraConstants.kWidth, CameraConstants.kHeight);
    camera.setFPS(CameraConstants.fps);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
