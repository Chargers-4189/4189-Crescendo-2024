// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.Scanner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class PlayBack extends Command {
  /** Creates a new PlayBack. */
  private DriveSubsystem swerveController;
  private Onboarder onboarder;
  private Shooter shooter;
  private GenericEntry onRed;
  private SendableChooser<File> recSelector;

  private double controlLeftY;
  private double controlLeftX;
  private double controlRightX;
  private double onboarderSpeed;
  private double shooterSpeed;

  private File rFile;
  private Scanner sc;

  public PlayBack(DriveSubsystem swerveController, Onboarder onboarder, Shooter shooter, SendableChooser<File> RecSelector, GenericEntry alliancebox) {
    this.swerveController = swerveController;
    this.onboarder = onboarder;
    this.shooter = shooter;

    this.recSelector = RecSelector;
    this.onRed = alliancebox;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveController, onboarder, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      rFile = new File(recSelector.getSelected().toString());
      sc = new Scanner(rFile);
      System.out.println("Playback Started with: " + recSelector.getSelected());

    } catch (IOException e) {
      System.out.println("Failed to read file: ");
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String cLine = sc.nextLine();
    String[] currentArray = cLine.split(",", 6);
    controlLeftY = Double.valueOf(currentArray[0]);
    controlLeftX = Double.valueOf(currentArray[1]);
    controlRightX = Double.valueOf(currentArray[2]);
    onboarderSpeed = Double.valueOf(currentArray[3]);
    shooterSpeed = Double.valueOf(currentArray[4]);

    if (!onRed.getBoolean(true)) {
      //controlLeftY *= 1; F&B
      controlLeftX *= -1;
      controlRightX *= -1;
    }

    this.swerveController.drive(
      MathUtil.applyDeadband(controlLeftY, OIConstants.kDriveDeadband),
      MathUtil.applyDeadband(controlLeftX, OIConstants.kDriveDeadband),
      MathUtil.applyDeadband(controlRightX, OIConstants.kDriveDeadband),
      false, true
    );
    onboarder.setOnboarder(-onboarderSpeed);
    shooter.setShooter(shooterSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sc.close();
    this.swerveController.drive(
      0,
      0,
      0,
      true, true
    );
    onboarder.setOnboarder(0);
    shooter.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (sc.hasNextLine()) {
      return false;
    } else {
      return true;
    }
  }
}
