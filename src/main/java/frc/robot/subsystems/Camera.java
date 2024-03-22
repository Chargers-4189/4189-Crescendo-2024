// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  private UsbCamera usbCamera = new UsbCamera("USB Camera", CameraConstants.kUSBPort);
  private ShuffleboardTab tab = Shuffleboard.getTab(CameraConstants.kShuffleboardTitle);
  public Camera() {
    tab.add(usbCamera).withPosition(CameraConstants.kPosY, CameraConstants.kPosY);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
