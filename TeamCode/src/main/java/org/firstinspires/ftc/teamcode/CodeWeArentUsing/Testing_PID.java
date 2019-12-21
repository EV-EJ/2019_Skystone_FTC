// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode.CodeWeArentUsing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PIDController;
import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PidDriveTrain;

@Autonomous(name="Testing PID", group="Exercises")
@Disabled
public class Testing_PID extends LinearOpMode
{
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction, rotation, distance;
    private boolean aButton, bButton;
    private PIDController pidRotate, pidDrive, pidDistance;
    private PidDriveTrain drive;

    // called when init button is  pressed.
    @Override
    public void runOpMode(){//} throws InterruptedException {
        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");

        /*LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        drive = new PidDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor,imu);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        pidRotate = new PIDController(.003, .00003, 0);

        pidDistance = new PIDController(.003, .00003, 0);

        // Set PID proportional value to produce non-zero correction value when robot veers off
        // straight line. P value controls how sensitive the correction is.
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        pidDistance.setSetpoint(1080);
        pidDistance.setOutputRange(-1, 1);
        pidDistance.setInputRange(0, 7000);
        pidDistance.reset();
        pidDistance.enable();

        // drive until end of period.

        telemetry.addData("Mode", "DOES THIS WORK?");
        telemetry.update();


        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        while (opModeIsActive())
        {
            // Use PID with imu input to drive in a straight line.

            telemetry.addData("Mode", "MIHAR IS A BOT");
            telemetry.update();

            drive.DriveForward(0.3);

            //drive.DriveForwardPID(5);

            /*telemetry.addData("Mode", "yes");
            telemetry.update();
            correction = pidDrive.performPID(getAngle());

            int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            distance = pidDistance.performPID(encode);

            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", correction);
            telemetry.addData("4 turn rotation", rotation);
            telemetry.addData("5 encode", encode);
            telemetry.addData("6 distance", distance);
            telemetry.update();

            /* set power levels.
            LFMotor.setPower(power - correction);
            LBMotor.setPower(power - correction);
            RBMotor.setPower(power + correction);
            RFMotor.setPower(power + correction);*/

            // set power levels.

            /*LFMotor.setPower(distance - correction);
            LBMotor.setPower(distance - correction);
            RBMotor.setPower(distance + correction);
            RFMotor.setPower(distance + correction);*/
            /*do {
                telemetry.addData("Mode", "hhheNG");
                telemetry.update();
                correction = pidDrive.performPID(getAngle());

                int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

                distance = pidDistance.performPID(encode);

                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.addData("5 encode", encode);
                telemetry.addData("6 distance", distance);
                telemetry.update();

            /* set power levels.
            LFMotor.setPower(power - correction);
            LBMotor.setPower(power - correction);
            RBMotor.setPower(power + correction);
            RFMotor.setPower(power + correction);*/

                // set power levels.
                //drive.DriveForward(distance);
                /*LFMotor.setPower(distance - correction);
                LBMotor.setPower(distance - correction);
                RBMotor.setPower(distance + correction);
                RFMotor.setPower(distance + correction);*/
            //} while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy());


            /*LFMotor.setPower(correction);
            LBMotor.setPower(correction);
            RFMotor.setPower(correction);
            RFMotor.setPower(correction);*/

            // We record the sensor values because we will test them in more than
            // one place with time passing between those places. See the lesson on
            // Timing Considerations to know why.

            /*aButton = gamepad1.a;
            bButton = gamepad1.b;

            if (aButton || bButton)
            {
                // backup.
                LFMotor.setPower(-power);
                LBMotor.setPower(-power);
                RFMotor.setPower(-power);
                RBMotor.setPower(-power);

                sleep(500);

                // stop.
                LFMotor.setPower(0);
                LBMotor.setPower(0);
                RFMotor.setPower(0);
                RBMotor.setPower(0);*/

                // turn 90 degrees right.
                //if (aButton) rotate(-90, power);

                // turn 90 degrees left.
                //if (bButton) rotate(90, power);
            //}
        }

        // turn the motors off.
        /*LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);*/
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                LFMotor.setPower(power);
                LBMotor.setPower(power);
                RFMotor.setPower(-power);
                RBMotor.setPower(-power);
                sleep(100);
            }

            do
            {
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LFMotor.setPower(-power);
                LBMotor.setPower(-power);
                RFMotor.setPower(power);
                RBMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LFMotor.setPower(-power);
                LBMotor.setPower(-power);
                RFMotor.setPower(power);
                RBMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        LFMotor.setPower(0);
        LBMotor.setPower(0);
        RFMotor.setPower(0);
        RBMotor.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}