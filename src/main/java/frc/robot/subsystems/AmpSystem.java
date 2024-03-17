// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSystem extends SubsystemBase {
  /** Creates a new AmpSystem. */
  private double encoderTicks;
  private double restLimit = 0;
  private double extendLimit = Constants.AmpSystemConstants.kEncoderMaxPosition;
  private double actuatorPower = 0;

  private Encoder encoder = new Encoder(Constants.AmpSystemConstants.kEncoderADIO, Constants.AmpSystemConstants.kEncoderBDIO);
  private DigitalInput ampSensor = new DigitalInput(Constants.AmpSystemConstants.kAmpSensorDIO);

  private WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.AmpSystemConstants.kAcuatorCanID);
  private WPI_TalonSRX roller = new WPI_TalonSRX(Constants.AmpSystemConstants.kRollerCanID);

  public AmpSystem() {
    actuator.configContinuousCurrentLimit(10, 1);
  }

  public void setRoller(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }
  public void setActuate(double power) {
    if (power > 0) {
      if (encoderTicks < restLimit) {
        this.actuatorPower = power;
      }else{
        this.actuatorPower = 0;
      }
    } else if (power < 0) {
      if (encoderTicks > extendLimit) {
        this.actuatorPower = power;
      } else {
        this.actuatorPower = 0;
      }
    } else {
        this.actuatorPower = 0;
    }
    actuator.set(ControlMode.PercentOutput, this.actuatorPower);
  }

  public boolean getAmpSensor() {
    return this.ampSensor.get();
  }
  public double getEncoderValue() {
    this.encoderTicks = encoder.get();
    return this.encoderTicks;
  }
  public void resetEncoder(){
   encoder.reset();
  }

  public boolean actuateToAmp() {
    double ratioToPos = this.encoderTicks / extendLimit;

    if (ratioToPos > 0.95) {
      setActuate(0);
      return true;
    } else if (ratioToPos > 0.8) {
      setActuate(-0.3);
    } else if (ratioToPos > 0.5) {
      setActuate(0.2);
    } else {
      setActuate(0.4);
    }
    return false;
  }

  public boolean actuateToRest() {
    double ratioToPos = this.encoderTicks / extendLimit;

    if (ratioToPos < 0.05) {
      setActuate(0);
      return true;
    } else if (ratioToPos < 0.2) {
      setActuate(0.3);
    } else if (ratioToPos < 0.5) {
      setActuate(-0.2);
    } else {
      setActuate(-0.4);
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Amp System - Encoder: " + getEncoderValue());
  }
}
