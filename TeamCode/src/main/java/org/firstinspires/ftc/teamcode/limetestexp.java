package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Limelight: final", group = "Vision")
public class limetestexp extends LinearOpMode {

    private DcMotor rotate;
    private Limelight3A limelight;

    /* ===== TUNING ===== */
    private static final double kP = 0.01;
    private static final double MAX_POWER = 0.25;
    private static final double TX_DEADBAND = 1.2; // Slightly wider to stop "hunting"

    // Low-Pass Filter: 0.1 means 10% new data, 90% old data.
    // Lower = smoother but slower. Higher = faster but twitchier.
    private static final double GAIN = 0.2;
    private double smoothedTx = 0;

    private int targetTagID = 20;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            double rotatePower = 0.0;
            LLResult result = limelight.getLatestResult();

            // 1. ID Selection
            if (gamepad1.left_bumper && !lastLeftBumper) targetTagID = Math.max(20, targetTagID - 1);
            if (gamepad1.right_bumper && !lastRightBumper) targetTagID = Math.min(25, targetTagID + 1);
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            boolean correctTagVisible = false;
            double distanceZ = 0;

            if (result != null && result.isValid()) {
                // Check if our specific ID is in the frame
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetTagID) {
                        correctTagVisible = true;
                        Pose3D pose = fr.getRobotPoseTargetSpace();
                        if (pose != null) distanceZ = Math.abs(pose.getPosition().z);
                        break;
                    }
                }

                if (correctTagVisible) {
                    // 2. APPLY LOW-PASS FILTER
                    // smoothedTx = (current_reading * GAIN) + (previous_smoothed * (1 - GAIN))
                    double rawTx = result.getTx();
                    smoothedTx = (rawTx * GAIN) + (smoothedTx * (1.0 - GAIN));

                    // 3. STABLE CALCULATION
                    if (Math.abs(smoothedTx) > TX_DEADBAND) {
                        rotatePower = smoothedTx * kP;
                    }

                    rotatePower = Math.max(-MAX_POWER, Math.min(MAX_POWER, rotatePower));
                }
            } else {
                // If we lose the tag, reset the filter so it doesn't "jump" when it reappears
                smoothedTx = 0;
            }

            /* ===== MANUAL OVERRIDE ===== */
            if (gamepad1.dpad_up) rotatePower = 0.2;
            else if (gamepad1.dpad_down) rotatePower = -0.2;

            rotate.setPower(rotatePower);

            // Telemetry for monitoring the "Smoothing"
            telemetry.addData("Selected ID", targetTagID);
            telemetry.addData("Tag Visible", correctTagVisible);
            telemetry.addData("Raw Tx", result != null ? result.getTx() : 0);
            telemetry.addData("Smoothed Tx", "%.2f", smoothedTx);
            telemetry.addData("Distance", "%.2f", distanceZ," m");
            telemetry.update();
        }
    }
}