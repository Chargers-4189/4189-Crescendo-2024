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
  private Encoder encoder = new Encoder(Constants.NoteAcuatorConstants.kEncoderADIO, Constants.NoteAcuatorConstants.kEncoderBDIO);
  private DigitalInput ampSensor = new DigitalInput(Constants.NoteAcuatorConstants.kAmpSensorDIO);

  private WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kAcuatorCanID);
  private WPI_TalonSRX roller = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kRollerCanID);

  public AmpSystem() {
    actuator.configContinuousCurrentLimit(10, 1);
  }

  public void setRoller(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }
  public void setActuate(double power) {
    actuator.set(ControlMode.PercentOutput, power);
  }

  public boolean getAmpSensor() {
    return this.ampSensor.get();
  }
  public void resetEncoder(){
   encoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
