package org.firstinspires.ftc.teamcode.Autonomous.DriveUsingPIDAndEncoders;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CodeWeArentUsing.FourEncoderDriveTrain;
import org.firstinspires.ftc.teamcode.DriveTrainAndPID.EncoderAndPIDDriveTrain;

//Autonomous program when facing crater

@Autonomous (name = "Blue_Build_Front")
//@Disabled
public class Blue_Foundation_Front_Final extends LinearOpMode {

    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo, foundServo, foundServo2;
    EncoderAndPIDDriveTrain drive;
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

        drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);

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
            //driving to the center of the foundation
            drive.DriveForwardDistance(1,15);
            drive.StrafeRightDistance(1,30);
            //grabbing the foundation
            foundServo.setPosition(0.6);
            foundServo2.setPosition(0.8);
            sleep(1000);
            //moving to the building site, and turning it
            drive.StrafeLeftDistance(1,50);
            drive.TurnLeftDistance(1,50);
            drive.StrafeRightDistance(1,20);
            drive.DriveForwardDistance(1, 15);
            //releasing the foundation
            foundServo.setPosition(0.4);
            foundServo2.setPosition(0.6);
            sleep(1000);
            //going under the bridge
            drive.StrafeLeftDistance(1,33);
            drive.DriveBackwardDistance(0.5,15);
        }
    }
}