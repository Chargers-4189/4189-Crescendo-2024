package frc.utils;

import edu.wpi.first.wpilibj.Timer;

public class Alarm {
    private double initTime;
    private double timeout;

    public Alarm(double seconds){
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + seconds;
    }

    public boolean hasTriggered(){
        initTime = Timer.getFPGATimestamp();
        return initTime > timeout;
    }
}
