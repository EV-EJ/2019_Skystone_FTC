package org.firstinspires.ftc.teamcode.DriveTrainAndPID;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class EncoderAndPIDDriveTrain {
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    private PIDController pidRotate;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    public EncoderAndPIDDriveTrain(DcMotor m_LFMotor, DcMotor m_LBMotor, DcMotor m_RFMotor, DcMotor m_RBMotor, BNO055IMU m_imu){
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

        pidRotate = new PIDController(.003, .00003, 0);

        //Run using encoders
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);

        this.imu = m_imu;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
    }

    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {

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

    public void DriveForward(double power) {
        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
    }

    public void StopDriving() {

        DriveForward(0);
    }


    //Drive forward using encoders
    public void DriveForwardDistance(double power, double distance)  {

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) ((LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        DriveForward(power);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            DriveForward(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void TurnLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(-power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
    }


    public void TurnLeftTime(double power, long time) throws InterruptedException {

        TurnLeft(power);
        Thread.sleep(time);
    }

    public void TurnLeftDegrees(double power, double degrees) throws InterruptedException {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        do {
            power = pidRotate.performPID(getAngle());
            TurnLeft(power);
        } while (!pidRotate.onTarget());

        // turn the motors off.
        StopDriving();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void TurnLeftDistance(double power, double distance)   {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(-encoderDistance);
        LBMotor.setTargetPosition(-encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnLeft(power);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            TurnLeft(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void DriveBackward(double power) {

        DriveForward(-power);
    }


    public void DriveBackwardDistance(double power, double distance)  {

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(-encoderDistance);
        LBMotor.setTargetPosition(-encoderDistance);
        RFMotor.setTargetPosition(-encoderDistance);
        RBMotor.setTargetPosition(-encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveBackward(power);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            DriveBackward(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void TurnRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(-power);
        RBMotor.setPower(-power);
    }

    public void TurnRightTime(double power, long time) throws InterruptedException {

        TurnRight(power);
        Thread.sleep(time);
    }

    public void TurnRightDegrees(double power, double degrees) throws InterruptedException {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        while (getAngle() == 0) {
            TurnRight(power);
            sleep(100);
        }

        do {
            power = pidRotate.performPID(getAngle());
            TurnLeft(power);
        } while (!pidRotate.onTarget());

        // turn the motors off.
        StopDriving();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void TurnRightDistance(double power, double distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(-encoderDistance);
        RBMotor.setTargetPosition(-encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnRight(power);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            TurnRight(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void StrafeRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(-power);
        RFMotor.setPower(-power);
        RBMotor.setPower(power);
    }


    public void StrafeRightDistance(double power, double distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(encoderDistance);
        LBMotor.setTargetPosition(-encoderDistance);
        RFMotor.setTargetPosition(-encoderDistance);
        RBMotor.setTargetPosition(encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        StrafeRight(power);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            StrafeRight(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void StrafeLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(-power);
    }

    public void StrafeLeftDistance(double power, double distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (int) (LFMotor.getCurrentPosition() + distance * 90);

        //Set target position
        LFMotor.setTargetPosition(-encoderDistance);
        LBMotor.setTargetPosition(encoderDistance);
        RFMotor.setTargetPosition(encoderDistance);
        RBMotor.setTargetPosition(-encoderDistance);

        //set run to position mode
        LFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RBMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        StrafeLeft(power);


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            StrafeLeft(power);
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
