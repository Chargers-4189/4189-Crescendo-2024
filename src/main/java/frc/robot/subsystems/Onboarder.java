// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Onboarder extends SubsystemBase {
  /** Creates a new Onboarder. */
  private DigitalInput bumperSensor = new DigitalInput(Constants.OnboarderConstants.kIntakeBeamDIO);
  private DigitalInput shooterSensor = new DigitalInput(Constants.OnboarderConstants.kOutakeBeamDIO);
  private WPI_VictorSPX onboardMotor = new WPI_VictorSPX(Constants.OnboarderConstants.konboardMotorCANID);
  private TalonSRX OnboarderLight = new TalonSRX(Constants.OnboarderConstants.kOnboarderLightCANID);
  private double maxBrightness = 1;
  private int timer = 0;

  private double onboarderPower = 0;

  private ShuffleboardTab tab = Shuffleboard.getTab(Constants.ShuffleboardConstants.kAutonomousTab);
  private final GenericEntry booleanbox = tab.add("Intake", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  
  public Onboarder() {}

  public boolean getBumperSensor(){
    return !this.bumperSensor.get();
  }
  public boolean getShooterSensor(){
    return !this.shooterSensor.get();
  }

  public void setOnboarder(double power) {
    this.onboarderPower = -power;
    onboardMotor.set(ControlMode.PercentOutput, this.onboarderPower);
  }
  public double getOnboarderSpeed() {
    return this.onboarderPower;
  }
  
  public void setDriverLight() {
    if(getShooterSensor()){
      OnboarderLight.set(TalonSRXControlMode.PercentOutput, maxBrightness);
    }else if(getBumperSensor()){
      if ((timer % 25) < 12.5) { // 1/2 seconds flash
        OnboarderLight.set(TalonSRXControlMode.PercentOutput, maxBrightness);
      } else {
        OnboarderLight.set(TalonSRXControlMode.PercentOutput, 0);
      }
    }else{
      OnboarderLight.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    booleanbox.setBoolean(!this.bumperSensor.get());
    setDriverLight();
    timer++;
    System.out.println(this.getOnboarderSpeed());
  }
}
