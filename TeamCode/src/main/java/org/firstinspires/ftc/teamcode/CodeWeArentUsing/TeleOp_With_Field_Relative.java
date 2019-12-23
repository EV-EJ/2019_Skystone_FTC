package org.firstinspires.ftc.teamcode.CodeWeArentUsing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="TeleOp_FieldRelative", group="Iterative TeamCode")
@Disabled
public class TeleOp_With_Field_Relative extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor, armMotor2,  clawMotor;
    private DcMotorEx LFMotor, LBMotor, RFMotor, RBMotor;
    private DigitalChannel limitSwitch;
    private Servo rotateServo, clawServo, foundServo, foundServo2;
    private boolean fieldRelativeMode = false;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        LFMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = (DcMotorEx) hardwareMap.get(DcMotor.class, "RB Motor");
        armMotor = hardwareMap.get(DcMotor.class, "Arm Motor 1");
        armMotor2 = hardwareMap.get(DcMotor.class, "Arm Motor 2");
        clawMotor = hardwareMap.get(DcMotor.class,"Claw Up Motor");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "Limit Stop");
        rotateServo = hardwareMap.get(Servo.class, "Rotate Servo");
        clawServo = hardwareMap.get(Servo.class, "Claw Servo");
        foundServo = hardwareMap.get(Servo.class, "found servo");
        foundServo2 = hardwareMap.get(Servo.class, "found servo 2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");


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

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu.initialize(parameters);

        resetAngle();

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

        double LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;
        float slidesValue;

        if (gamepad1.back) {
            fieldRelativeMode = !fieldRelativeMode;
        }

        telemetry.addData("FieldRelative?", fieldRelativeMode);

        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;


        if (fieldRelativeMode){
            telemetry.addData("hey", "hi?");
            double angle = getAngle();
            double tempX = (xValue * Math.cos(Math.toRadians(angle))) - (yValue * Math.sin(Math.toRadians(angle)));
            yValue = (xValue * Math.sin(Math.toRadians(angle))) + (yValue * Math.cos(Math.toRadians(angle)));
            xValue = tempX;
        }

        LFPower = -yValue + turnValue + xValue;
        LBPower = -yValue + turnValue - xValue;
        RBPower = -yValue - turnValue + xValue;
        RFPower = -yValue - turnValue - xValue;


        slidesValue = gamepad2.left_stick_y;

        
        LFMotor.setPower(Range.clip(LFPower, -1, 1));
        LBMotor.setPower(Range.clip(LBPower, -1, 1));
        RFMotor.setPower(Range.clip(RFPower, -1, 1));
        RBMotor.setPower(Range.clip(RBPower, -1, 1));

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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double getAngle()
    {

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

}
