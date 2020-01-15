package org.firstinspires.ftc.teamcode.Autonomous.TestingConcepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous (name = "Drive Motor PID Calibration")
@Disabled
public class PIDF_Calibration extends LinearOpMode {
    private DcMotorEx LFMotor, LBMotor, RFMotor, RBMotor;
    private double currentVelocityLF, currentVelocityLB, currentVelocityRF, currentVelocityRB;
    @Override

    public void runOpMode() {
        LFMotor  = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotorEx.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotorEx.class, "RB Motor");

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);

        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("WHY HAVEN'T YOU STARTED?", "START!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            /*LFMotor.setVelocityPIDFCoefficients(1.196, 0.1196, 0, 11.96);
            LFMotor.setPositionPIDFCoefficients(5.0);

            LBMotor.setVelocityPIDFCoefficients(1.179, 0.1179, 0, 11.79);
            LBMotor.setPositionPIDFCoefficients(5.0);

            RBMotor.setVelocityPIDFCoefficients(1.187, 0.1187, 0, 11.87);
            RBMotor.setPositionPIDFCoefficients(5.0);

            RFMotor.setVelocityPIDFCoefficients(1.205, 0.1205, 0, 12.05);
            RFMotor.setPositionPIDFCoefficients(5.0);*/

            LFMotor.setVelocity(2740/2);
            LBMotor.setVelocity(2780/2);
            RFMotor.setVelocity(2720/2);
            RBMotor.setVelocity(2760/2);

            currentVelocityLB = LBMotor.getVelocity();
            currentVelocityLF = LFMotor.getVelocity();
            currentVelocityRB = RBMotor.getVelocity();
            currentVelocityRF = RFMotor.getVelocity();

            telemetry.addData("current velocity LB", currentVelocityLB);
            telemetry.addData("current velocity LF", currentVelocityLF);
            telemetry.addData("current velocity RB", currentVelocityRB);
            telemetry.addData("current velocity RF", currentVelocityRF);
            telemetry.update();

        }

    }

}