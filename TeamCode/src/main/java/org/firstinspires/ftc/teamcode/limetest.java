package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


@TeleOp(name="limetest", group="Linear OpMode")
//@Disabled
public class limetest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor rotate = null;
    //    private DcMotor leftBackDrive = null;

    private Limelight3A limelight;


    @Override
    public void runOpMode() {


        rotate = hardwareMap.get(DcMotor.class, "rotate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        telemetry.addData("Status", limelight.isConnected());
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // testing the intake and kicker
            if (gamepad1.dpad_up) {
                rotate.setPower(0.1);     // forward hi
//            } else if (gamepad1.dpad_down) {
//                intake.setPower(1);    // reverse
            } else if (gamepad1.dpad_down){
                rotate.setPower(-0.1);
            }
            else{
                rotate.setPower(0);
            }



//                telemetry.addData("Triangle(Y)", gamepad1.y);
//                telemetry.addData("Square (X)", gamepad1.x);
//                telemetry.addData("Servo Position", servo.getPosition());
//                telemetry.update();

            LLResult result = limelight.getLatestResult();

            if ((result.getTx()) < 0){
                rotate.setPower(-0.1);
            } else if ((result.getTx()) < 0){
                rotate.setPower(0.1);
            }

            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("power", 0.1);
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                }
            }           // Send calculated power to wheels


            telemetry.update();

            if (isStopRequested()) {
                return;
            }
        }
    }}





