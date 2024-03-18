// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX topMotor = new TalonFX(Constants.ShooterConstants.kTopShooterCANID);
  private TalonFX bottomMotor = new TalonFX(Constants.ShooterConstants.kBottomShooterCANID);

  public Shooter() {}

  public void setShooter(double power) {
    power = power * 9;
    this.topMotor.setControl(new VelocityVoltage(-power));
    this.bottomMotor.setControl(new VelocityVoltage(power));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
