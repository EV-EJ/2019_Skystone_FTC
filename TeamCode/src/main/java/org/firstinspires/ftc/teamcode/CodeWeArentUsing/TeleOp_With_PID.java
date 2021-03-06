package org.firstinspires.ftc.teamcode.CodeWeArentUsing;
//imported packages for out code
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


//This file is a TeleOp file, which means that this will be using the joysticks in the 30 min period

@TeleOp(name="TeleOp_Test_PID", group="Iterative TeamCode")
@Disabled
public class TeleOp_With_PID extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor armMotor, armMotor2, clawMotor; //clawMotor, clawMotor2, LFMotor, LBMotor, RFMotor, RBMotor, ;
    private DcMotorEx LFMotor, LBMotor, RFMotor, RBMotor;
    private DigitalChannel limitSwitch;
    private Servo rotateServo, clawServo, foundServo, foundServo2;
    private double speed = 1;
    private double expectedSpeed = 0.1;
    private double currSpeed = 0;
    private ElapsedTime timer;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor2.setDirection(DcMotor.Direction.FORWARD);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.FORWARD);//reversed 10-4-2019 Dhruva
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor2.setDirection(DcMotor.Direction.REVERSE);
        clawMotor.setDirection(DcMotor.Direction.REVERSE);
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
        rotateServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);
        foundServo2.setDirection(Servo.Direction.REVERSE);
        foundServo.setDirection(Servo.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        float   LFPower, LBPower, RFPower, RBPower, xValue, turnValue, yValue;
        float slidesValue;
        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        yValue = gamepad1.left_stick_y;
        turnValue = gamepad1.right_stick_x;
        xValue = gamepad1.left_stick_x;

        LFPower = -yValue + turnValue + xValue;
        LBPower = -yValue + turnValue - xValue;
        RBPower = -yValue - turnValue + xValue;
        RFPower = -yValue - turnValue - xValue;

        if (LFPower == 0){
            timer.reset();
        }

        slidesValue = gamepad2.left_stick_y;

        if (gamepad1.a){
            speed = 0.2;
        } else{
            speed = 1;
        }
        /*if (expectedSpeed > speed){
            expectedSpeed = speed;
        }*/
        expectedSpeed = speed;
        telemetry.addData("Power for data", LFPower);
        /*if (expectedSpeed < LFPower) {

        }*/
        //The wheels in our code
        LFMotor.setPower(Range.clip(LFPower, -expectedSpeed, expectedSpeed));
        LBMotor.setPower(Range.clip(LBPower, -expectedSpeed, expectedSpeed));
        RFMotor.setPower(Range.clip(RFPower, -expectedSpeed, expectedSpeed));
        RBMotor.setPower(Range.clip(RBPower, -expectedSpeed, expectedSpeed));

        telemetry.addData("LF Motor",LFMotor.getVelocity());
        telemetry.addData("LB Motor",LBMotor.getVelocity());
        telemetry.addData("RF Motor",RFMotor.getVelocity());
        telemetry.addData("RB Motor",RBMotor.getVelocity());

        /*LFMotor.setPower(Range.clip(LFPower, -1, 1));
        LBMotor.setPower(Range.clip(LBPower, -1, 1));
        RFMotor.setPower(Range.clip(RFPower, -1, 1));
        RBMotor.setPower(Range.clip(RBPower, -1, 1));*/
        //This is the lift mechanism
        if (slidesValue == 0){
            clawMotor.setPower(-0.1);
        } else if (limitSwitch.getState() || slidesValue >= 0) {
            clawMotor.setPower(Range.clip(slidesValue, -0.6, 0.07));
        }
        //This is the claw that will pick up our brick
        if (gamepad1.x) {
            clawServo.setPosition(0.9);
        }
        if (gamepad1.y) {
            clawServo.setPosition(1);
        }
        telemetry.addData("found",foundServo.getPosition());
        telemetry.addData("found2",foundServo2.getPosition());
        //the intake meachanism for our code
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
        //Rotate arm 180 degrees
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
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}