package frc.utils;

import edu.wpi.first.wpilibj.Timer;

public class Alarm {
    private double initTime;
    private double timeout;
    private double duration;

    public Alarm(double seconds){
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + seconds;
        duration = seconds;
    }

    public boolean hasTriggered(){
        initTime = Timer.getFPGATimestamp();
        return initTime > timeout;
    }

    public void initAlarm(){
        initTime = Timer.getFPGATimestamp();
        timeout = Timer.getFPGATimestamp() + duration;
    }
}
