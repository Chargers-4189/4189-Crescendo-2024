// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSystem;

public class AmpOuttake extends Command {
  /** Creates a new AmpOuttake. */
  private boolean isFinished;
  private double initTime = 0;
  private double stopTime = 0;
  private double duration = 2;

  private AmpSystem ampSystem;

  public AmpOuttake(AmpSystem ampSystem) {
    this.ampSystem = ampSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    initTime = Timer.getFPGATimestamp();
    stopTime = Timer.getFPGATimestamp() + duration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    initTime = Timer.getFPGATimestamp();
    ampSystem.setRoller(1);

    // Need to tie with Amp Sensor for better accuracy
    if (initTime >= stopTime) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSystem.setRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
