// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpSystem;

public class ActuateToRest extends Command {
  /** Creates a new ActuateToRest. */
  private boolean isFinished;
  private double initTime = 0;
  private double timeoutTime = 0;
  private double duration = Constants.AmpSystemConstants.kAcuateTimeoutLimit;

  private AmpSystem ampSystem;

  public ActuateToRest(AmpSystem ampSystem) {
    this.ampSystem = ampSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    initTime = Timer.getFPGATimestamp();
    timeoutTime = Timer.getFPGATimestamp() + duration;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double ratioToPos = ampSystem.getEncoderValue() / Constants.AmpSystemConstants.kEncoderMaxPosition;
    initTime = Timer.getFPGATimestamp();

    if (ratioToPos < 0.05) {
      ampSystem.setActuate(0);
      isFinished = true;
    } else if (ratioToPos < 0.5) {
      ampSystem.setActuate(-0.2);
    } else {
      ampSystem.setActuate(-0.4);
    }

    if (initTime >= timeoutTime) {
      ampSystem.setActuate(0);
      ampSystem.disableMotor();
      throw new Error("ActuateToRest has exceeded timeout limit");
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
