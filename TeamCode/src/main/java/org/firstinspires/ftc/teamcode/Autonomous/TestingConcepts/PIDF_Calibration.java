package org.firstinspires.ftc.teamcode.Autonomous.TestingConcepts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous (name = "Drive Motor PID Calibration")
//@Disabled
public class PIDF_Calibration extends LinearOpMode {
    DcMotorEx LFMotor, LBMotor, RFMotor, RBMotor;
    double currentVelocityLF, currentVelocityLB, currentVelocityRF, currentVelocityRB;
    double maxVelocityLF = 0.0, maxVelocityLB = 0.0, maxVelocityRF = 0.0, maxVelocityRB = 0.0;
    @Override

    public void runOpMode() {
        LFMotor  = hardwareMap.get(DcMotorEx.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotorEx.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotorEx.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotorEx.class, "RB Motor");

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            currentVelocityLB = LBMotor.getVelocity();
            currentVelocityLF = LFMotor.getVelocity();
            currentVelocityRB = RBMotor.getVelocity();
            currentVelocityRF = RFMotor.getVelocity();

            if (currentVelocityLB > maxVelocityLB) {
                maxVelocityLB = currentVelocityLB;
            }
            if (currentVelocityLF > maxVelocityLF) {
                maxVelocityLF = currentVelocityLF;
            }
            if (currentVelocityRB > maxVelocityRB) {
                maxVelocityRB = currentVelocityRB;
            }
            if (currentVelocityRF > maxVelocityRF) {
                maxVelocityRF = currentVelocityRF;
            }

            telemetry.addData("current velocity LB", currentVelocityLB);
            telemetry.addData("maximum velocity LB", maxVelocityLB);
            telemetry.addData("current velocity LF", currentVelocityLF);
            telemetry.addData("maximum velocity LF", maxVelocityLF);
            telemetry.addData("current velocity RB", currentVelocityRB);
            telemetry.addData("maximum velocity RB", maxVelocityRB);
            telemetry.addData("current velocity RF", currentVelocityRF);
            telemetry.addData("maximum velocity RF", maxVelocityRF);
            telemetry.update();

        }

    }

}