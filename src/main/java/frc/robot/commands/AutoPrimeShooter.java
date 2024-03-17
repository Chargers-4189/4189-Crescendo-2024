// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class AutoPrimeShooter extends Command {
  private Shooter shooter;
  private Onboarder onboarder;
  private AmpSystem ampSystem;

  /** Creates a new PrimeShooter. */
  public AutoPrimeShooter(Shooter shooter, Onboarder onboarder, AmpSystem ampSystem) {
    this.shooter = shooter;
    this.onboarder = onboarder;
    this.ampSystem = ampSystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, onboarder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ampSystem.getAmpSensor()) {
      if (!onboarder.getShooterSensor()) {
        onboarder.setOnboarder(0);
        shooter.setShooter(1);
      } else {
        onboarder.setOnboarder(-0.5);
      }
    } else {
      onboarder.setOnboarder(0);
      shooter.setShooter(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    onboarder.setOnboarder(0);
    shooter.setShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
