package frc.utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class AbsoluteEncoderSetpoint {
    
    private DutyCycleEncoder encoder;
    private double setpoint;
    private boolean above;
    public AbsoluteEncoderSetpoint(DutyCycleEncoder encoder, double setPoint, boolean above){
        this.encoder = encoder;
        this.setpoint = setPoint;
        this.above = above;
    }

    public double getAbsolutePosition(){
        return encoder.getAbsolutePosition();
    }

    public boolean isTriggered(){
        if(above){
            return getAbsolutePosition() > setpoint;
        } else {
            return setpoint > getAbsolutePosition();
        }
    }
}