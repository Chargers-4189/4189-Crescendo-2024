// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpSystem;
import frc.utils.Alarm;

public class DisableActuator extends Command {
  /** Creates a new ActuateToAmp. */
  private AmpSystem ampSystem;
  private boolean isFinished;
  private Alarm timeout = new Alarm(Constants.AmpSystemConstants.kAcuateTimeoutLimit);

  public DisableActuator(AmpSystem ampSystem) {
    this.ampSystem = ampSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    timeout.initAlarm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampSystem.setActuate(-1);

    if (timeout.hasTriggered()) {
      ampSystem.setActuate(0);
      System.err.println("ActuateToAmp has exceeded timeout limit");
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSystem.setActuate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
