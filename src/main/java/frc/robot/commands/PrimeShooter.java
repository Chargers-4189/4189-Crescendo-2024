// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;
import frc.utils.Alarm;

public class PrimeShooter extends Command {
  private boolean isFinished;
  private Alarm timer = new Alarm(1.5);

  private Shooter shooter;
  private Onboarder onboarder;

  /** Creates a new PrimeShooter. */
  public PrimeShooter(Shooter shooter, Onboarder onboarder) {
    this.shooter = shooter;
    this.onboarder = onboarder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, onboarder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    timer.initAlarm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooter(Constants.ShooterConstants.kShooterPowerValue);

    if (!onboarder.getShooterSensor() || timer.hasTriggered()) {
      onboarder.setOnboarder(0);
      isFinished = true;

    } else {
      onboarder.setOnboarder(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    onboarder.setOnboarder(0);
    shooter.setShooter(Constants.ShooterConstants.kShooterPowerValue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
