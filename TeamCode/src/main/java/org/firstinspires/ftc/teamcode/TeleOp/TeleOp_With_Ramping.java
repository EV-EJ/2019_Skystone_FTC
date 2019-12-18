package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TeleOp_Ramping", group="Iterative TeamCode")
/*@Disabled*/
public class TeleOp_With_Ramping extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor, armMotor2, LFMotor, LBMotor, RFMotor, RBMotor, clawMotor; //clawMotor, clawMotor2, ;
    private DigitalChannel limitSwitch;
    private Servo rotateServo, clawServo, foundServo, foundServo2;
    private double speed = 1;


    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");
        armMotor = hardwareMap.get(DcMotor.class, "Arm Motor 1");
        armMotor2 = hardwareMap.get(DcMotor.class, "Arm Motor 2");
        clawMotor = hardwareMap.get(DcMotor.class,"Claw Up Motor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "Limit Stop");
        rotateServo = hardwareMap.get(Servo.class, "Rotate Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        foundServo = hardwareMap.get(Servo.class, "found servo");
        foundServo2 = hardwareMap.get(Servo.class, "found servo 2");


        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        clawMotor.setDirection(DcMotor.Direction.REVERSE);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rotateServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        foundServo2.setDirection(Servo.Direction.REVERSE);
        foundServo.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {

        float   LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;
        float slidesValue;


        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        LFPower = Range.clip(-yValue + turnValue + xValue,-1,1);
        LBPower = Range.clip(-yValue + turnValue - xValue,-1,1);
        RBPower = Range.clip(-yValue - turnValue + xValue,-1,1);
        RFPower = Range.clip(-yValue - turnValue - xValue,-1,1);

        if (LFPower < 0){
            LFPower = (float) -Math.pow(Math.abs(LFPower),2);
        } else if (LFPower > 0){
            LFPower = (float) Math.pow(Math.abs(LFPower),2);
        }

        if (LBPower < 0){
            LBPower = (float) -Math.pow(Math.abs(LBPower),2);
        } else if (LBPower > 0){
            LBPower = (float) Math.pow(Math.abs(LBPower),2);
        }

        if (RFPower < 0){
            RFPower = (float) -Math.pow(Math.abs(RFPower),2);
        } else if (RFPower > 0){
            RFPower = (float) Math.pow(Math.abs(RFPower),2);
        }

        if (RBPower < 0){
            RBPower = (float) -Math.pow(Math.abs(RBPower),2);
        } else if (RBPower > 0){
            RBPower = (float) Math.pow(Math.abs(RBPower),2);
        }

        slidesValue = gamepad2.left_stick_y;

        if (gamepad1.a){
            speed = 0.1;
        } else{
            speed = 1;
        }

        LFMotor.setPower(Range.clip(LFPower, -speed, speed));
        LBMotor.setPower(Range.clip(LBPower, -speed, speed));
        RFMotor.setPower(Range.clip(RFPower, -speed, speed));
        RBMotor.setPower(Range.clip(RBPower, -speed, speed));

        if (slidesValue == 0){
            clawMotor.setPower(-0.1);
        } else if (limitSwitch.getState() || slidesValue >= 0) {
            clawMotor.setPower(Range.clip(slidesValue, -0.6, 0.07));
        }

        if (gamepad1.x) {
            clawServo.setPosition(0.9);
        }
        if (gamepad1.y) {
            clawServo.setPosition(1);
        }
        telemetry.addData("found",foundServo.getPosition());
        telemetry.addData("found2",foundServo2.getPosition());

        if(gamepad1.right_bumper){
            armMotor.setPower(0.5);
            armMotor2.setPower(0.5);
        } else if(gamepad1.left_bumper){
            armMotor.setPower(-0.5);
            armMotor2.setPower(-0.5);
        } else{
            armMotor.setPower(0);
            armMotor2.setPower(0);
        }

        if (gamepad2.right_bumper){
            rotateServo.setPosition(Servo.MIN_POSITION);
        } else if(gamepad2.left_bumper){
            rotateServo.setPosition(Servo.MAX_POSITION);
        }
        if (gamepad2.x) {
            telemetry.addData("x","pressed");
            foundServo.setPosition(0.4);
            foundServo2.setPosition(0.6);
        }
        if (gamepad2.y) {
            telemetry.addData("y","pressed");
            foundServo.setPosition(0.6);
            foundServo2.setPosition(0.8);

        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }


    @Override
    public void stop() {
    }

}