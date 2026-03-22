package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@TeleOp(name = "Robot: Final testing", group = "Main")
public class FinalRobotTest extends LinearOpMode {

    /* Hardware Members */
    private DcMotor shooter1, shooter2;


    /* Limelight Tuning Constants */
    // Proportional gain: scales how much rotate power is applied per degree of tx offset
    private static final double kP = 0.03;
    // Maximum power allowed to the rotate motor when vision-tracking (prevents violent spinning)
    private static final double MAX_ROTATE_POWER = 0.4;
    // Minimum tx (horizontal) offset (degrees) before rotation kicks in — prevents jitter when nearly centered
    private static final double TX_DEADBAND = 0.5;
    // Low-pass filter weight: higher = more responsive but noisier; lower = smoother but slower to react
    private static final double GAIN = 0.2;
    // Exponential moving average of tx — smooths out noisy Limelight readings frame-to-frame
    private double smoothedTx = 0;
    // AprilTag ID currently being tracked; bumpers cycle this between 20–25
    private int targetTagID = 20;

    /* State tracking for toggle/button debouncing */
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain



        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);



        telemetry.addData("Status", "Initialized - Ready to Fly");
        telemetry.update();

        waitForStart();

        /*
        * all button mappings:
        * Left stick: forward and backward, strafe left and right
        * right stick, rotate (change heading) left and right
        * a: intake
        * b: shooter
        * x: kicker on intake
        * y: pusher on shooter
        * bumpers:  limelight targettagID
        * dpad up/down: override limelight rotation
        * right trigger then bumpers: change hood angle */

        while (opModeIsActive()) {

            // --- SECTION 1: MECANUM DRIVE ---
            // Negate left_stick_y because pushing the stick forward gives a negative value on FTC gamepads
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Mecanum wheel mixing: each wheel gets a combination of axial, lateral, and yaw
            // The signs come from the physical roller orientation of mecanum wheels
            double lf = axial + lateral + yaw;
            double rf = axial - lateral - yaw;
            double lb = axial - lateral + yaw;
            double rb = axial + lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }


            // Shooter (Motor) - Spin while B is pressed
            if (gamepad1.b) shooter1.setPower(1.0);
            else shooter1.setPower(0);
            if (gamepad1.x) shooter2.setPower(1.0);
            else shooter2.setPower(0);



            // --- SECTION 4: TELEMETRY ---
            telemetry.addData("Target ID", targetTagID);
            telemetry.addData("Drive", "LF:%.2f RF:%.2f", lf, rf);
            telemetry.addData("smoothedtx: ",smoothedTx);
            telemetry.update();
        }
    }
}