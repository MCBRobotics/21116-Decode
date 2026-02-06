package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight rotating working", group = "Vision")
public class limetest extends LinearOpMode {

    private DcMotor rotate;
    private Limelight3A limelight;

    /* ===== TUNING ===== */
    private static final double kP = 0.01;
    private static final double MAX_POWER = 0.25;
    private static final double TX_DEADBAND = 1.0;

    @Override
    public void runOpMode() {

        rotate = hardwareMap.get(DcMotor.class, "rotate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.setMsTransmissionInterval(20);

        telemetry.addLine("Limelight init complete");
        telemetry.update();

        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {

            double rotatePower = 0.0;

            LLResult result = limelight.getLatestResult();

            telemetry.addLine("===== LIMELIGHT DEBUG =====");

            if (result == null) {
                telemetry.addLine("Result: NULL (no data yet)");
            } else {
                telemetry.addData("Target Valid", result.isValid());
                telemetry.addData("tx (horizontal error)", result.getTx());
                telemetry.addData("ty (vertical error)", result.getTy());
                telemetry.addData("ta (target area)", result.getTa());

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    telemetry.addData("BotPose", botpose.toString());
                }

                /* ===== AUTO TRACK ARM ===== */
                if (result.isValid()) {
                    double tx = result.getTx();

                    if (Math.abs(tx) > TX_DEADBAND) {
                        rotatePower = tx * kP;
                    } else {
                        rotatePower = 0.0;
                    }

                    rotatePower = Math.max(
                            -MAX_POWER,
                            Math.min(MAX_POWER, rotatePower)
                    );
                }
            }

            /* ===== MANUAL OVERRIDE ===== */
            if (gamepad1.dpad_up) {
                rotatePower = 0.2;
                telemetry.addLine("MANUAL: DPAD UP");
            } else if (gamepad1.dpad_down) {
                rotatePower = -0.2;
                telemetry.addLine("MANUAL: DPAD DOWN");
            }

            rotate.setPower(rotatePower);

            telemetry.addLine("===== ARM DEBUG =====");
            telemetry.addData("Rotate Power", rotatePower);

            telemetry.update();
        }
    }
}
