package org.firstinspires.ftc.teamcode.CodeWeArentUsing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PIDController;

public class RampingPidDriveTrain {
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;
    private PIDController pidDrive;
    private double power, encode, previous_power = 0;

    public RampingPidDriveTrain(DcMotor m_LFMotor, DcMotor m_LBMotor, DcMotor m_RFMotor, DcMotor m_RBMotor){
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

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
    }

    public double rampingUp(double previous_power, double power){
        if (previous_power < power && power > 0){
            return 0;
        } else {
            return power;
        }
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
        pidDrive.reset();
        pidDrive.setSetpoint(inches * 90.37);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            power = pidDrive.performPID(encode);

            DriveForward(power);

            previous_power = power;
        }

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

    public void TurnLeftDistance(double power, int distance)   {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = LFMotor.getCurrentPosition() + distance * 90;

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

    //Drive Backward using pid may not work. needs to be tested
    public void DriveBackwardPID(double inches){
        pidDrive.reset();
        pidDrive.setSetpoint(inches * 90.37);

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {
            int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

            double power = pidDrive.performPID(encode);

            DriveBackward(power);
        }

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

    public void TurnRightDistance(double power, int distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = LFMotor.getCurrentPosition() + distance * 90;

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


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {//wait until target position is reached
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


    public void StrafeRightDistance(double power, int distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = LFMotor.getCurrentPosition() + distance * 90;

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


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {//wait until target position is reached
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

    public void StrafeLeftDistance(double power, int distance) {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = LFMotor.getCurrentPosition() + distance * 90;

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


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {//wait until target position is reached
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
