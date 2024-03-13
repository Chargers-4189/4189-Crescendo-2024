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

public class NoteActuator extends SubsystemBase {
  /** Creates a new NoteActuator. */
  private double liftPower = 0;
  private Encoder encoder = new Encoder(Constants.NoteAcuatorConstants.kEncoderADIO, Constants.NoteAcuatorConstants.kEncoderBDIO);
  
  private WPI_TalonSRX liftMotor = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kScissorLiftCanID);
  private WPI_TalonSRX actuator = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kAcuatorCanID);
  private WPI_TalonSRX roller = new WPI_TalonSRX(Constants.NoteAcuatorConstants.kRollerCanID);

  private DigitalInput ScissorLimitSensor = new DigitalInput(Constants.NoteAcuatorConstants.kScissorLimitDIO);

  public NoteActuator() {
    actuator.configContinuousCurrentLimit(10, 1); //prevents the motor from burning out from stalling
  }

  public void setRoller(double power) {
    roller.set(ControlMode.PercentOutput, power);
  }

  public void setActuate(double power) {
    actuator.set(ControlMode.PercentOutput, power);
  }

  public void setLift(double power) {
    // IF at limit and going up, stop.
    if (getScissorLimitSensor() && power > 0) {
      this.liftPower = 0;
    } else {
      this.liftPower = power;
    } 

    liftMotor.set(ControlMode.PercentOutput, this.liftPower);
  }

  public boolean getScissorLimitSensor() {
    return this.ScissorLimitSensor.get();
  }

  public void resetEncoder(){
   encoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println(encoder.get());
  }
}
