package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

/**
 * TurretAimTag23_NoPriority
 * - Aims motor "rotate" at AprilTag ID 23 WITHOUT using Limelight's "Priority Tag".
 * - Reads per-tag fiducial results and selects the item with id==23.
 * - Uses tx (deg) from that fiducial as horizontal error and applies a simple P-control.
 *
 * Requirements:
 *   Limelight pipeline type = Fiducial Markers (36h11). (No Priority Tag needed.)
 */
@TeleOp(name = "Turret: Aim Tag 23 (No Priority)", group = "Vision")
public class TurretAimTag23_NoPriority extends LinearOpMode {

    // Hardware
    private Limelight3A limelight;
    private DcMotor rotate;

    // REV Core Hex: 288 ticks/rev at output → 0.8 ticks/deg
    private static final double TICKS_PER_REV = 288.0;
    private static final double TICKS_PER_DEG = TICKS_PER_REV / 360.0; // 0.8

    // soft limits 
    private static final double MAX_LEFT_DEG  =  150.0; // +deg from zero
    private static final double MAX_RIGHT_DEG = -150.0; // -deg from zero
    private static final int LEFT_LIMIT_TICKS  = (int)( MAX_LEFT_DEG  * TICKS_PER_DEG);
    private static final int RIGHT_LIMIT_TICKS = (int)( MAX_RIGHT_DEG * TICKS_PER_DEG);

    // Limelight tuning
    private static final int    PIPELINE_INDEX = 0;    // Your AprilTag pipeline index
    private static final int    TARGET_ID      = 23;   // The AprilTag ID we care about

    // Turret controller tuning
    private static final double kP        = 0.02;  // motor power per degree of tx
    private static final double MIN_PWR   = 0.08;  // overcome Core Hex static friction
    private static final double MAX_PWR   = 0.30;  // keep gentle
    private static final double DEADBAND  = 1.0;   // degrees considered "square"

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Hardware map ---
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // we set power but can read position

        // --- Limelight init (FTC pattern: start → getLatestResult → isValid) ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();  // begin polling frames  (standard FTC Limelight usage)

        telemetry.addLine("Turret Aim (No Priority) ready. Press PLAY.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double cmd = 0.0;
            double txDegFor23 = Double.NaN;

            // 1) Grab the latest result and confirm it's usable
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                // 2) Get all fiducials seen this frame and pick ID 23
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                for (LLResultTypes.FiducialResult f : tags) {
                    if (f.getId() == TARGET_ID) {
                        // Per-tag horizontal error (deg) for this fiducial
                        txDegFor23 = f.getTx();
                        break;
                    }
                }
            }

            // 3) Turn the turret only if Tag 23 is detected this frame
            if (!Double.isNaN(txDegFor23)) {
                double err = txDegFor23;
                if (Math.abs(err) <= DEADBAND) {
                    cmd = 0.0; // squared
                } else {
                    cmd = kP * err;
                    // ensure we break static friction
                    if (cmd > 0) cmd = Math.max(cmd,  MIN_PWR);
                    else         cmd = Math.min(cmd, -MIN_PWR);
                    // cap
                    cmd = Range.clip(cmd, -MAX_PWR, MAX_PWR);
                }
            } else {
                // No Tag 23 this frame: do nothing (hold)
                cmd = 0.0;
            }

            // 4) Enforce optional soft limits (protects hard-stops)
            int pos = rotate.getCurrentPosition(); // ticks (Core Hex: 288 ticks/rev)
            if (pos >= LEFT_LIMIT_TICKS && cmd > 0)  cmd = 0.0;
            if (pos <= RIGHT_LIMIT_TICKS && cmd < 0) cmd = 0.0;

            // 5) Apply
            rotate.setPower(cmd);

            // Telemetry
            telemetry.addData("Tag23 tx (deg)", Double.isNaN(txDegFor23) ? "N/A" : txDegFor23);
            telemetry.addData("MotorCmd", cmd);
            telemetry.addData("Pos (ticks)", pos);
            telemetry.addData("SoftLimit L/R", LEFT_LIMIT_TICKS + " / " + RIGHT_LIMIT_TICKS);
            telemetry.update();

            sleep(10);
        }

        rotate.setPower(0.0);
        // limelight.stop(); // optional
    }
}