package test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import team25core.MonitorMotorTask;
import team25core.Robot;
import team25core.RobotEvent;

/**
 * Created by Bella Heinrichs on 11/04/2017.
 */

@Autonomous(name = "Violet TEST Motor", group = "AutoTest")
//@Disabled
public class VioletMotorTest extends Robot {

    private DcMotor motor;
    private int positionFL;

    @Override
    public void handleEvent(RobotEvent e) {

    }
    /*
        Port 0  || frontLeft
        Port 1  || backLeft
        Port 2  || frontRight
        Port 3  || backRight
     */
    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("backLeft");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() {
        motor.setPower(0.5);
        addTask(new MonitorMotorTask(this, motor));
    }

}
