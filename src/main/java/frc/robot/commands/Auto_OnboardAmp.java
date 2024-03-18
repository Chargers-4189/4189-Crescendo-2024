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
public class Auto_OnboardAmp extends SequentialCommandGroup {
  /** Creates a new AutoOnboardAmp. */
  public Auto_OnboardAmp(AmpSystem ampSystem, Onboarder onboarder, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
      addCommands(new ActuateToAmp(ampSystem), new AmpOnboard(onboarder, shooter, ampSystem));
    } catch (Exception e) {
      addCommands(new CancelAmpSystem(onboarder, shooter, ampSystem));
      System.out.println("Failed to AutoOnboard");
      e.printStackTrace();
    }
  }
}
