package frc.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoderSetpoint {
    
    private DutyCycleEncoder encoder;
    private double setpoint;

    public AbsoluteEncoderSetpoint(DutyCycleEncoder encoder, double setPoint){
        this.encoder = encoder;
        this.setpoint = setPoint;
    }

    public double getAbsolutePosition(){
        return encoder.getAbsolutePosition();
    }

    public double getPercentageAbove(){
        return getAbsolutePosition() / setpoint;
    }

    public double getPercentageBelow(){
        return setpoint / getAbsolutePosition();
    }

    public boolean isTopTriggered(){
        return getPercentageAbove() >= 1;
    }

    public boolean isBottomTriggered(){
        return getPercentageBelow() >= 1;
    }
}