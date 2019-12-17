package org.firstinspires.ftc.teamcode.DriveTrainAndPID;

import com.qualcomm.robotcore.hardware.DcMotor;

public class NoEncoderDriveTrain {
    private DcMotor LFMotor, LBMotor, RFMotor, RBMotor;

    public NoEncoderDriveTrain(DcMotor m_LFMotor, DcMotor m_LBMotor, DcMotor m_RFMotor, DcMotor m_RBMotor){
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

        //Run using encoders
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
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

    public void DriveBackward(double power) {

        DriveForward(-power);
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


    public void StrafeRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(-power);
        RFMotor.setPower(-power);
        RBMotor.setPower(power);
    }


    public void StrafeLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(-power);
    }
}
