package org.firstinspires.ftc.teamcode.CodeWeArentUsing.TestingConcepts;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeWeArentUsing.FourEncoderDriveTrain;
import org.firstinspires.ftc.teamcode.DriveTrainAndPID.PIDController;


//Back up Auton that goes to the wall side of the bridge, and parks there
@Config
@Autonomous (name = "TESTING PID")
@Disabled
public class Testing_PID extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, clawMotor;
    DigitalChannel limitSwitch;
    Servo rotateServo, clawServo;
    FourEncoderDriveTrain drive;
    private PIDController pidDrive;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode(){
        // Initialize the hardware variables.
        LFMotor  = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor  = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor  = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor  = hardwareMap.get(DcMotor.class, "RB Motor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Wheels on the robot funtions
        drive = new FourEncoderDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor);
        pidDrive = new PIDController(kP, kI, kD);

        pidDrive.setOutputRange(-1, 1);
        pidDrive.setInputRange(-100000, 100000);
        pidDrive.setTolerance(100);
        pidDrive.setSetpoint(500);
        pidDrive.reset();
        pidDrive.enable();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        runtime.reset();
        waitForStart();

        //Running the code
        LFMotor.getCurrentPosition();
        while (opModeIsActive()) {
            do {
                int encode = (LFMotor.getCurrentPosition() + LBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + RBMotor.getCurrentPosition()) / 4;

                double power = pidDrive.performPID(encode);

                telemetry.addData("encode", encode);
                telemetry.addData("LF Motor",LFMotor.getCurrentPosition());
                telemetry.addData("LB Motor",LBMotor.getCurrentPosition());
                telemetry.addData("RF Motor",RFMotor.getCurrentPosition());
                telemetry.addData("RB Motor",RBMotor.getCurrentPosition());
                telemetry.addData("y", runtime);
                telemetry.addData("x",power);
                telemetry.log();
                telemetry.update();

                drive.DriveForward(power);
            } while (LFMotor.isBusy() && LBMotor.isBusy() && RFMotor.isBusy() && RBMotor.isBusy());
        }
    }

}