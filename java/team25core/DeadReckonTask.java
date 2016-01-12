package team25core;

/*
 * FTC Team 25: cmacfarl, September 01, 2015
 */

import com.qualcomm.robotcore.util.RobotLog;

public class DeadReckonTask extends RobotTask {

    public enum EventKind {
        SEGMENT_DONE,
        PATH_DONE,
    }

    public class DeadReckonEvent extends RobotEvent {

        public EventKind kind;
        public int segment_num;

        public DeadReckonEvent(RobotTask task, EventKind k, int segment_num)
        {
            super(task);
            kind = k;
            this.segment_num = segment_num;
        }

        @Override
        public String toString()
        {
            return (super.toString() + "DeadReckon Event " + kind + " " + segment_num);
        }
    }

    protected DeadReckon dr;
    protected int num;
    protected boolean waiting;
    SingleShotTimerTask sst;

    public DeadReckonTask(Robot robot, DeadReckon dr)
    {
        super(robot);

        this.num = 0;
        this.dr = dr;
        this.waiting = false;
    }

    @Override
    public void start()
    {
        // TODO: ??
    }

    @Override
    public void stop()
    {
        robot.removeTask(this);
    }

    @Override
    public boolean timeslice()
    {
        /*
         * If runPath returned true it consumed a segment, send an event
         * back to the robot.
         */
        robot.telemetry.addData("Segment: ", num);

        if (dr.runPath()) {
            robot.queueEvent(new DeadReckonEvent(this, EventKind.SEGMENT_DONE, num++));
            return false;
        }

        if (dr.done()) {
            robot.queueEvent(new DeadReckonEvent(this, EventKind.PATH_DONE, num));
            /*
             * Make sure it's stopped.
             */
            RobotLog.i("251 Done with path, stopping all");
            dr.stop();
            return true;
        } else {
            return false;
        }
    }
}