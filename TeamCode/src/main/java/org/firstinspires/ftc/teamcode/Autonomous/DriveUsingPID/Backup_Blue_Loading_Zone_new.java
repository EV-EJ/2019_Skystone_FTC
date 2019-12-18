package org.firstinspires.ftc.teamcode.Autonomous.DriveUsingPID;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PidDriveTrain;

//Autonomous program when facing crater

@Autonomous (name = "Blue_Load")
@Disabled
public class Backup_Blue_Loading_Zone_new extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo, foundServo, foundServo2;
    PidDriveTrain drive;
    BNO055IMU imu;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");
        clawMotor = hardwareMap.get(DcMotor.class,"Claw Up Motor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "Limit Stop");
        rotateServo = hardwareMap.get(Servo.class, "Rotate Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        foundServo = hardwareMap.get(Servo.class, "found servo");
        foundServo2 = hardwareMap.get(Servo.class, "found servo 2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        drive = new PidDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor,imu);

        //Reverse the right motors to move forward based on their orientation on the robot
        clawMotor.setDirection(DcMotor.Direction.REVERSE);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rotateServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        foundServo2.setDirection(Servo.Direction.REVERSE);
        foundServo.setDirection(Servo.Direction.FORWARD);



        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        waitForStart();

        LFMotor.getCurrentPosition();
        if (opModeIsActive()) {
            drive.StrafeRightPID(15);

            while (limitSwitch.getState()) {
                clawMotor.setPower(-0.6);
            }
            clawMotor.setPower(-0.1);

            rotateServo.setPosition(Servo.MAX_POSITION);
            clawServo.setPosition(0);
            sleep(2000);
            clawMotor.setPower(0);
            sleep(1000);
            clawServo.setPosition(1);
            sleep(1000);
            while (limitSwitch.getState()) {
                clawMotor.setPower(-0.6);
            }
            clawMotor.setPower(-0.1);
            rotateServo.setPosition(Servo.MIN_POSITION);
            sleep(1000);
            clawMotor.setPower(0);
            sleep(950);
            clawMotor.setPower(-0.1);

            drive.StrafeLeftPID( 20);
            drive.StrafeRightPID(4);
            drive.DriveForwardPID(30 );

            while (limitSwitch.getState()) {
                clawMotor.setPower(-0.6);
            }
            clawMotor.setPower(-0.1);

            rotateServo.setPosition(Servo.MAX_POSITION);
            sleep(2000);
            clawServo.setPosition(0);
            sleep(500);
            rotateServo.setPosition(Servo.MIN_POSITION);
            sleep(1000);
            clawMotor.setPower(0);
            sleep(1000);
            drive.DriveBackwardPID(15);

        }
    }
}