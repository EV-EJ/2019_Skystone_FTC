package org.firstinspires.ftc.teamcode.Autonomous.DriveUsingTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.DriveTrainAndPID.FourEncoderDriveTrain;

//Autonomous program when facing crater

@Autonomous (name = "Blue_Load")
@Disabled
public class Foundation_2 extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo, foundServo, foundServo2;
    FourEncoderDriveTrain drive;


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

        drive = new FourEncoderDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor);

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
            drive.DriveBackward(1);
            sleep(500);
            drive.StrafeLeft(1);
            sleep(1000);
            foundServo.setPosition(0.6);
            foundServo2.setPosition(0.8);
            sleep(1000);
            drive.StrafeRight(1);
            sleep(1250);
            drive.TurnLeft(1);
            sleep(250);
            drive.StrafeLeft(1);
            sleep(300);
            foundServo.setPosition(0.4);
            foundServo2.setPosition(0.6);
            sleep(1000);
            drive.StrafeRight(1);
            sleep(1100);
            drive.DriveBackward(0.5);
            sleep(100);
            drive.DriveForward(1);
            sleep(2000);

        }
    }
}