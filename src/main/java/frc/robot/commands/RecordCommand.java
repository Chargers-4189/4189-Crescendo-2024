// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RecordPlaybackConstants;
import frc.robot.subsystems.AmpSystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class RecordCommand extends Command {
  /** Creates a new RecordCommand. */
  private Onboarder onboarder;
  private Shooter shooter;
  private Joystick leftShaft;
  private Joystick rightShaft;

  private FileWriter rFile;
  private SendableChooser<File> RecSelector;
  private GenericEntry recFileName;
  private File recordFile;
  private int fileCount = 0;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;

  public RecordCommand(Onboarder onboarder, Shooter shooter, AmpSystem ampSystem, Joystick leftShaft, Joystick rightShaft, SendableChooser<File> RecSelector, GenericEntry FileName) {
    this.leftShaft = leftShaft;
    this.rightShaft = rightShaft;
    
    this.onboarder = onboarder;
    this.shooter = shooter;

    this.RecSelector = RecSelector;
    this.recFileName = FileName;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
        recordFile = new File(RecordPlaybackConstants.kRecordDirectory,recFileName.getString("rec"+fileCount)+"."+RecordPlaybackConstants.kFileType);
        rFile = new FileWriter(recordFile);
        System.out.println("Recording at: " + recordFile);
      } catch (IOException e) {
        System.out.println("Failed to create file: ");
        e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    controlLeftY = leftShaft.getY();
    controlLeftX = leftShaft.getX();
    controlRightX = rightShaft.getX();

    onboarderSpeed = onboarder.getOnboarderSpeed();
    shooterSpeed = shooter.getShooterSpeed();

    try {
      rFile.append(
        String.valueOf(controlLeftY) + "," +
        String.valueOf(controlLeftX) + "," +
        String.valueOf(controlRightX) + "," +
        String.valueOf(onboarderSpeed) + "," +
        String.valueOf(shooterSpeed) + "," +
        "\n"
      );
    } catch (IOException e) {
     System.err.println("An error occurred while recording: ");
      e.printStackTrace();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      rFile.close();
    } catch (IOException e) {
      System.out.println("Failed to end record: ");
      e.printStackTrace();
    }

    RecSelector.addOption(recordFile.getName(), recordFile);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
