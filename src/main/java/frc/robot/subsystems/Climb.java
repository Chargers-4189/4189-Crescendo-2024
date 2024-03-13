// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private double climbPower = 0;

  private DigitalInput restPosition = new DigitalInput(Constants.ClimbConstants.kRestLimitDIO);
  private DigitalInput extendPosition = new DigitalInput(Constants.ClimbConstants.kExtendLimitDIO);
  private WPI_TalonSRX climbMotor = new WPI_TalonSRX(Constants.ClimbConstants.kClimbMotorID);

  public Climb() {}

  public void setClimb(double power) {
    // STOP in the direction of sensor if detected
    if (getRestSensor() && power < 0) {
      climbPower = 0;
    } else if (getExtendedSensor() && power > 0) {
      climbPower = 0;
    } else {
      climbPower = power;
    }

    this.climbMotor.set(ControlMode.PercentOutput, climbPower);
  }

  public boolean getRestSensor() {
    return this.restPosition.get();
  }
  public boolean getExtendedSensor() {
    return this.extendPosition.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
