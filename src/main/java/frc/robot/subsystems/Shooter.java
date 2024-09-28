// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX topMotor = new TalonFX(Constants.ShooterConstants.kTopShooterCANID);
  private TalonFX bottomMotor = new TalonFX(Constants.ShooterConstants.kBottomShooterCANID);

  public Shooter() {}

  public void setShooter(double power) {
    power = power * 9.0;
    double powerTop = power + 2;
    double powerBottom = power -2;
    if (power != 0) {
      this.topMotor.setControl(new VoltageOut(-powerTop));
      this.bottomMotor.setControl(new VoltageOut(powerBottom));
    } else {
      this.topMotor.setControl(new VoltageOut(0));
      this.bottomMotor.setControl(new VoltageOut(0));
    }
  }

  public void setShooterLowPower(double power) {
    this.topMotor.set(-power);
    this.bottomMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
