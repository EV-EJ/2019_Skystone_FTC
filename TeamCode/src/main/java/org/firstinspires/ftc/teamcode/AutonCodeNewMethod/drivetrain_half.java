package org.firstinspires.ftc.teamcode.AutonCodeNewMethod;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class drivetrain_half {
    private static DcMotor LFMotor, LBMotor, RFMotor, RBMotor;

    public drivetrain_half(DcMotor m_LFMotor, DcMotor m_LBMotor, DcMotor m_RFMotor, DcMotor m_RBMotor){
        this.LBMotor = m_LBMotor;
        this.LFMotor = m_LFMotor;
        this.RBMotor = m_RBMotor;
        this.RFMotor = m_RFMotor;

        //Run using encoders
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public static void DriveForward(double power) {
        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
    }

    public static void StopDriving() {

        DriveForward(0);
    }


    //Drive forward using encoders
    public static void DriveForwardDistance(double power, int distance)  {
        //LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90;

        //DriveForward(power);

        double av = 0;

        do {

            int encodercountLF = LFMotor.getCurrentPosition();
            int encodercountRB = RBMotor.getCurrentPosition();

            av = (encodercountLF + encodercountRB) / 2;

            DriveForward(power);
        } while (av < encoderDistance);

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public static void TurnLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(-power);
        RFMotor.setPower(power);
        RBMotor.setPower(power);
    }


    public static void TurnLeftTime(double power, long time) throws InterruptedException {

        TurnLeft(power);
        Thread.sleep(time);
    }

    public static void TurnLeftDistance(double power, int distance)   {
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


        while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy()) {//wait until target position is reached
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public static void DriveBackward(double power) {

        DriveForward(-power);
    }


    public static void DriveBackwardDistance(double power, int distance)  {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90;

        //DriveForward(power);

        double av = 0;

        do {

            int encodercountLF = LFMotor.getCurrentPosition();
            int encodercountRB = RBMotor.getCurrentPosition();

            av = (encodercountLF + encodercountRB) / 2;

            DriveBackward(power);
        } while (av > encoderDistance);

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public static void TurnRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(power);
        RFMotor.setPower(-power);
        RBMotor.setPower(-power);
    }

    public static void TurnRightTime(double power, long time) throws InterruptedException {

        TurnRight(power);
        Thread.sleep(time);
    }

    public static void TurnRightDistance(double power, int distance) {
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
        }

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public static void StrafeRight(double power) {

        LFMotor.setPower(power);
        LBMotor.setPower(-power);
        RFMotor.setPower(-power);
        RBMotor.setPower(power);
    }


    public static void StrafeRightDistance(double power, int distance)  {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90;

        //DriveForward(power);

        double av = 0;

        do {

            int encodercountLF = LFMotor.getCurrentPosition();
            int encodercountRB = RBMotor.getCurrentPosition();

            av = (encodercountLF + encodercountRB) / 2;

            StrafeRight(power);
        } while (av < encoderDistance);

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public static void StrafeLeft(double power) {

        LFMotor.setPower(-power);
        LBMotor.setPower(power);
        RFMotor.setPower(power);
        RBMotor.setPower(-power);
    }

    public static void StrafeLeftDistance(double power, int distance)  {
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
        //Ticks per inch = 1136/12.57 (approximately 90.37)
        int encoderDistance = (LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition())/2 + distance * 90;

        //DriveForward(power);

        double av = 0;

        do {

            int encodercountLF = LFMotor.getCurrentPosition();
            int encodercountRB = RBMotor.getCurrentPosition();

            av = (encodercountLF + encodercountRB) / 2;

            StrafeLeft(power);
        } while (av < encoderDistance);

        //Stop and change modes back to normal
        StopDriving();
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
}
