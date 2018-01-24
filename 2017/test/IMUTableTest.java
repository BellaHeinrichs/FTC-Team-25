/* Copyright (c) 2015 Craig MacFarlane

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Craig MacFarlane nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import team25core.DeadReckonPath;
import team25core.DeadReckonTask;
import team25core.IMUTableConfiguration;
import team25core.Robot;
import team25core.RobotEvent;
import team25core.TwoWheelDirectDrivetrain;

@Autonomous(name="IMU Table Test", group="AutoTeam25")
@Disabled
public class IMUTableTest extends Robot
{
    private DcMotor frontRight;
    private DcMotor frontLeft;

    private DeadReckonTask deadReckonTask;
    private final static double STRAIGHT_SPEED = BellaConfiguration.STRAIGHT_SPEED;
    private final static double TURN_SPEED = BellaConfiguration.TURN_SPEED;
    private final static int TICKS_PER_INCH = BellaConfiguration.TICKS_PER_INCH;=
    private final static int TICKS_PER_DEGREE = BellaConfiguration.TICKS_PER_DEGREE;
    private DeadReckonPath deadReckon;
    private TwoWheelDirectDrivetrain drivetrain;

    private IMUSensorCriteria imuSensorCriteria;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    @Override
    public void handleEvent(RobotEvent e)
    {
        // Nothing.
    }

    @Override
    public void init()
    {
        frontRight = hardwareMap.dcMotor.get("rightMotor");
        frontLeft = hardwareMap.dcMotor.get("leftMotor");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drivetrain = new TwoWheelDirectDrivetrain(frontRight, frontLeft);

        deadReckon = new DeadReckonPath();
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        deadReckon.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, STRAIGHT_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.TURN, 75, TURN_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, STRAIGHT_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.TURN, 75, TURN_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, STRAIGHT_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.TURN, 75, TURN_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 15, STRAIGHT_SPEED);
        deadReckon.addSegment(DeadReckonPath.SegmentType.TURN, 75, TURN_SPEED);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imuSensorCriteria = new IMUSensorCriteria(imu, IMUTableConfiguration.MAX_TILT);
    }
    void composeTelemetry() {
        telemetry.addLine()
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
        telemetry.addLine()
                .addData("tilt", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.toDegrees(Math.acos(Math.cos(Math.toRadians(angles.secondAngle) *
                                        Math.cos(Math.toRadians(angles.thirdAngle))))));
                    }
                });
    }

    @Override
    public void start()
    {
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        //deadReckonTask = new DeadReckonTask(this, deadReckon, drivetrain, imuSensorCriteria);
        //addTask(deadReckonTask);

        final DeadReckonPath path = new DeadReckonPath();

        path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 10, IMUTableConfiguration.STRAIGHT_SPEED);


        this.addTask(new DeadReckonTask(this, path, drivetrain, imuSensorCriteria) {
            @Override
            public void handleEvent(RobotEvent e) {
                if (e instanceof DeadReckonEvent) {
                    DeadReckonEvent drEvent = (DeadReckonEvent) e;

                    if (drEvent.kind == EventKind.SENSOR_SATISFIED) {
                        RobotLog.i("Robot is STOPPED, reached maximum tilt level.");
                        path.stop();
                        path.addSegment(DeadReckonPath.SegmentType.STRAIGHT, 4, IMUTableConfiguration.REVERSE_SPEED);
                    } else if (drEvent.kind == EventKind.PATH_DONE){
                        RobotLog.i("Path done before tiltMax satisfied");
                    }
                }
            }
        });

    }

    public void stop()
    {
        if (deadReckonTask != null) {
            deadReckonTask.stop();
        }
    }
}
