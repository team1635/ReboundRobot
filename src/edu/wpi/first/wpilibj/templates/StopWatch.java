/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

/**
 *
 * @author Driver
 */
public class StopWatch {

    private long start;
    private int delay;
    
//    public StopWatch() {
//        this.delay = 1000;
//    }
    
    public StopWatch(int delay) {
        this.delay = delay;
    }

    public void start() {
        start = System.currentTimeMillis();
    }
    
    public void setDelay(int delay) {
        this.delay = delay;
    }

    public boolean delayExpired() {
        boolean retValue = false;

        if (System.currentTimeMillis() - start > delay) {
            retValue = true;
        }

        return retValue;
    }
    
}
