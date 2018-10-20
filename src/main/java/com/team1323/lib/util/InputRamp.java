package com.team1323.lib.util;

public class InputRamp{
    double output = 0.0;
    double rampRate;
    //Only ramp when the input is approaching the center.
    double center;
    double lastTimestamp = 0.0;
    boolean approachingCenter = false;

    public InputRamp(double center, double rate){
        this.center = center;
        rampRate = rate;
    }

    /**
     * 
     * @param rate Time from zero to full throttle
     */
    public void setRampRate(double rate){
        rampRate = rate;
    }

    public double update(double input, double timestamp){
        if(input == center && output != center){
            output += Math.signum(center - output) * ((timestamp - lastTimestamp) / rampRate);
        }else{
            output = input;
        }

        lastTimestamp = timestamp;

        return output;
    }
}