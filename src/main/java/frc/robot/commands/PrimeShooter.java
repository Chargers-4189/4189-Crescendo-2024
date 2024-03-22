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
  private Alarm endBackwind = new Alarm(3);
  private Alarm initAutoShoot = new Alarm(2);
  private Alarm startShooter = new Alarm(1);

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
    endBackwind.initAlarm();
    initAutoShoot.initAlarm();
    startShooter.initAlarm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (startShooter.hasTriggered()) {
      shooter.setShooter(Constants.ShooterConstants.kShooterPowerValue);
    }

    if (endBackwind.hasTriggered() || (!onboarder.getShooterSensor() && initAutoShoot.hasTriggered())) {
      shooter.setShooter(Constants.ShooterConstants.kShooterPowerValue);
      onboarder.setOnboarder(0);
      isFinished = true;

    } else {
      onboarder.setOnboarder(-0.35);
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
