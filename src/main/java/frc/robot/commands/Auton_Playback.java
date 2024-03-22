// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpSystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auton_Playback extends ParallelCommandGroup {
  /** Creates a new AUTO_Playback. */
  public Auton_Playback(AmpSystem ampSystem, DriveSubsystem swerve, Onboarder onboarder, Shooter shooter, SendableChooser<File> playbackFile, GenericEntry AllianceBox) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Auto_Shoot(ampSystem, shooter, onboarder), new PlayBack(swerve, onboarder, shooter, playbackFile, AllianceBox));
  }
}
