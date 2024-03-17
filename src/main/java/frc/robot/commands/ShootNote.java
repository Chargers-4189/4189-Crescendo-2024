// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class ShootNote extends Command {
  private boolean isFinished;
  private double initTime = 0;
  private double stopTime = 0;
  private double duration = 1;

  private Shooter shooter;
  private Onboarder onboarder;

  /** Creates a new ShootNote. */
  public ShootNote(Shooter shooter, Onboarder onboarder) {
    this.shooter = shooter;
    this.onboarder = onboarder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, onboarder);
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
    shooter.setShooter(Constants.ShooterConstants.kShooterPowerValue);
    onboarder.setOnboarder(1);

    if (initTime >= stopTime) {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooter(0);
    onboarder.setOnboarder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
