// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpSystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

public class CancelAll extends Command {
  private boolean isFinished;

  DriveSubsystem swerve;
  Shooter shooter;
  Onboarder onboarder;
  Climb climb;
  AmpSystem ampSystem;

  /** Creates a new CancelAll. */
  public CancelAll(DriveSubsystem swerve, Onboarder onboarder, Shooter shooter, Climb climb, AmpSystem ampSystem) {
    this.swerve = swerve;
    this.onboarder = onboarder;
    this.shooter = shooter;
    this.climb = climb;
    this.ampSystem = ampSystem;

    // Use addRequirements() here to declare subsystem dependencies.
  //addRequirements(swerve, onboarder, shooter, climb, ampSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    onboarder.setOnboarder(0.01);
    shooter.setShooter(0.01);
    climb.setClimb(0.01);
    swerve.drive(
      0.01,
      0.01,
      0.01,
      false,
      false
    );
    ampSystem.setRoller(0.01);
    ampSystem.setActuate(0.01);

    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    onboarder.setOnboarder(0);
    shooter.setShooter(0);
    climb.setClimb(0);
    swerve.drive(
      0,
      0,
      0,
      false,
      false
    );
    ampSystem.setRoller(0);
    ampSystem.setActuate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
