// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpSystem extends SubsystemBase {
  /** Creates a new AmpSystem. */
  private double encoderTicks;
  private double actuatorPower = 0;
  private boolean disableMotor = false;

  private DutyCycleEncoder encoder = new DutyCycleEncoder(Constants.AmpSystemConstants.kEncoderPWM);
  private DigitalInput ampSensor = new DigitalInput(Constants.AmpSystemConstants.kAmpSensorDIO);

  private WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.AmpSystemConstants.kAcuatorCanID);
  private WPI_VictorSPX roller = new WPI_VictorSPX(Constants.AmpSystemConstants.kRollerCanID);

  private double restLimit = Constants.AmpSystemConstants.kEncoderRestPosition;
  private double extendLimit = Constants.AmpSystemConstants.kEncoderAmpPosition;

  public AmpSystem() {
    actuator.configContinuousCurrentLimit(10, 1);
  }

  public void setRoller(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }
  public void setActuate(double power) {
    /*if (power > 0) {
      if (getEncoderValue() < restLimit) {
        this.actuatorPower = power;
      }else{
        this.actuatorPower = 0;
      }
    } else*/ if (power < 0) {
      if (getEncoderValue() > extendLimit) {
        this.actuatorPower = power;
      } else {
        this.actuatorPower = 0;
      }
    } else {
        this.actuatorPower = 0;
    }
    this.actuatorPower = power;
    if (!disableMotor) {
      actuator.set(ControlMode.PercentOutput, this.actuatorPower);
    }
  }

  public boolean getAmpSensor() {
    return this.ampSensor.get();
  }
  public double getEncoderValue() {
    this.encoderTicks = encoder.getAbsolutePosition();
    return this.encoderTicks;
  }
  public double getRestPosition() {
    return restLimit;
  }
  public void resetEncoder() {
    //restLimit = encoder.getAbsolutePosition();
    System.out.println("AmpSystem Encoder Reset");
  }

  public void disableMotor() {
    this.disableMotor = true;
    actuator.set(ControlMode.PercentOutput, 0);
  }
  public void enableMotor() {
    this.disableMotor = false;
    actuator.set(ControlMode.PercentOutput, 0);
  }

  public DutyCycleEncoder getEncoderObject() {
    return this.encoder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("Amp System - Encoder: " + encoder.getAbsolutePosition());
    //System.out.println(ampSensor.);
  }
}
