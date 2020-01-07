package org.firstinspires.ftc.teamcode.CodeWeArentUsing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PIDController;

import static java.lang.Thread.sleep;

public class PidDriveTrain {
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    private PIDController pidDrive, pidRotate;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    public PidDriveTrain(DcMotor m_LFMotor, DcMotor m_LBMotor, DcMotor m_RFMotor, DcMotor m_RBMotor, BNO055IMU m_imu){
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

        pidDrive = new PIDController(.003, .00003, 0);
        pidRotate = new PIDController(.003, .00003, 0);

        pidDrive.setOutputRange(-1, 1);
        pidDrive.setInputRange(-100000, 100000);
        pidDrive.reset();
        pidDrive.enable();

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

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle() {

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

    public void DriveForwardPID(double inches){
        double setpoint = 1136 / (4 * 3.14159265);
        pidDrive.setSetpoint(inches * setpoint);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        do {
            int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            double power = pidDrive.performPID(Math.abs(encode));

            DriveForward(power);
        } while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy());

    }


    public void TurnLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(-power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
    }

    public void TurnLeftDegrees(double power, int degrees) throws InterruptedException {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

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

    public void DriveBackward(double power) {

        DriveForward(-power);
    }

    //Drive Backward using pid may not work. needs to be tested
    public void DriveBackwardPID(double inches){
        pidDrive.reset();
        pidDrive.setSetpoint(-inches * 90.37);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            double power = pidDrive.performPID(Math.abs(encode));

            DriveBackward(power);
        }

    }


    public void TurnRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(-power);
        RBMotor.setPower(-power);
    }

    public void TurnRightDegrees(double power, int degrees) throws InterruptedException {
        resetAngle();

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

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

    public void StrafeRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(-power);
        RFMotor.setPower(-power);
        RBMotor.setPower(power);
    }

    public void StrafeRightPID(double inches){
        pidDrive.reset();
        pidDrive.setSetpoint(-inches * 90.37);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            int encode = (LFMotor.getCurrentPosition() - LBMotor.getCurrentPosition() - RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            double power = pidDrive.performPID(Math.abs(encode));

            StrafeRight(power);
        }

    }


    public void StrafeLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(-power);
    }

    public void StrafeLeftPID(double inches){
        pidDrive.reset();
        pidDrive.setSetpoint(-inches * 90.37);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            int encode = (LFMotor.getCurrentPosition() - LBMotor.getCurrentPosition() - RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            double power = pidDrive.performPID(Math.abs(encode));

            StrafeLeft(power);
        }

    }
}
