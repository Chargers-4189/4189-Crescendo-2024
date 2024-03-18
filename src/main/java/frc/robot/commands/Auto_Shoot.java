// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AmpSystem;
import frc.robot.subsystems.Onboarder;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_Shoot extends SequentialCommandGroup {
  /** Creates a new AutoShoot. */

  public Auto_Shoot(AmpSystem ampSystem, Shooter shooter, Onboarder onboarder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
      addCommands(new ActuateToAmp(ampSystem), new PrimeShooter(shooter, onboarder), new ShootNote(shooter, onboarder), new ActuateToRest(ampSystem));
    } catch (Exception e) {
      addCommands(new CancelAmpSystem(onboarder, shooter, ampSystem));
      System.err.println("Failed to AutoShoot");
      e.printStackTrace();
    }
  }
}
