// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpSystem;
import frc.utils.Alarm;

public class AutoToggleActuate extends Command {
  /** Creates a new AutoToggleActuate. */
  private boolean isFinished;
  private Alarm timeout = new Alarm(Constants.AmpSystemConstants.kAcuateTimeoutLimit);

  private AmpSystem ampSystem;
  private boolean restPosition;

  public AutoToggleActuate(AmpSystem ampSystem) {
    System.err.println("Hello WOrld");
    this.ampSystem = ampSystem;
    // Use addRequirements() here to declare subsystem dependencies.
  //addRequirements(ampSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (ampSystem.getEncoderValue() > 0.6) {
      restPosition = true;
    } else {
      restPosition = false;
    }
    isFinished = false;
    timeout.initAlarm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (restPosition) {

      if (ampSystem.getEncoderValue() <= Constants.AmpSystemConstants.kEncoderAmpPosition) {
        ampSystem.setActuate(0);
        isFinished = true;
      } else {
        ampSystem.setActuate(-1);
      }

    } else {

      if (ampSystem.getEncoderValue() >= ampSystem.getRestPosition()) {
        ampSystem.setActuate(0);
        isFinished = true;
      } else {
        ampSystem.setActuate(1);
      }

    }

    if (timeout.hasTriggered()) {
      ampSystem.setActuate(0);
      ampSystem.disableMotor();
      System.err.println("AutoToggleActuate has exceeded timeout limit");
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
