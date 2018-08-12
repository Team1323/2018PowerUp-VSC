package com.team1323.frc2018.auto;

import com.team1323.frc2018.auto.actions.Action;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.trajectory.timing.TimedState;
import java.util.List;
import java.util.ArrayList;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase {
    protected double m_update_rate = 1.0 / 50.0;
    protected boolean m_active = false;

    protected static TrajectoryGenerator.TrajectorySet trajectories = TrajectoryGenerator.getInstance().getTrajectorySet();

    protected static List<Trajectory<TimedState<Pose2dWithCurvature>>> paths = new ArrayList<>();
    public void addPath(Trajectory<TimedState<Pose2dWithCurvature>> path){
        paths.add(path);
    }
    public static List<Trajectory<TimedState<Pose2dWithCurvature>>> getPaths(){
        return paths;
    }


    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        m_active = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            System.out.println("Auto mode done, ended early");
            return;
        }

        done();
        System.out.println("Auto mode done");
    }

    public void done() {
    }

    public void stop() {
        m_active = false;
    }

    public boolean isActive() {
        return m_active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (m_update_rate * 1000.0);

            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

}
