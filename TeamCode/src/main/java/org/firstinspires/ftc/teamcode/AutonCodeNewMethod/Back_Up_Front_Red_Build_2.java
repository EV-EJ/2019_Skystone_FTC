package org.firstinspires.ftc.teamcode.AutonCodeNewMethod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

//Autonomous program when facing crater

@Autonomous (name = "Back_Up_Front_Red_Build")
//@Disabled
public class Back_Up_Front_Red_Build_2 extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo, foundServo, foundServo2;
    drivetrain drive;


    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        clawMotor = hardwareMap.get(DcMotor.class, "Claw Up Motor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "Limit Stop");
        rotateServo = hardwareMap.get(Servo.class, "Rotate Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        foundServo = hardwareMap.get(Servo.class, "found servo");
        foundServo2 = hardwareMap.get(Servo.class, "found servo 2");

        drive = new drivetrain(LFMotor, LBMotor, RFMotor, RBMotor);

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
            drive.DriveForwardDistance(1, 12);
            drive.StrafeRightDistance(1, 12);
        }
    }
}