package org.firstinspires.ftc.teamcode.UsingPID;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonCodeNewMethod.drivetrain;

//Autonomous program when facing crater

@Autonomous (name = "Blue_Build_Back")
//@Disabled
public class Blue_Foundation_Back_new extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo, foundServo, foundServo2;
    drivetrain_pid drive;
    BNO055IMU imu;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode() throws InterruptedException {
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

        drive = new drivetrain_pid(LFMotor, LBMotor, RFMotor, RBMotor, imu);

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
            drive.DriveBackwardPID(12);
            drive.StrafeLeftPID(30);
            foundServo.setPosition(0.6);
            foundServo2.setPosition(0.8);
            sleep(1000);
            drive.StrafeRightPID(40);
            drive.TurnLeftDegrees(1,90);
            drive.StrafeLeftPID(20);
            foundServo.setPosition(0.4);
            foundServo2.setPosition(0.6);
            sleep(1000);
            drive.StrafeRightPID(33);
            drive.DriveBackwardPID(5);
            //DriveBackwardDistance(0.5, 8);
        }
    }

}