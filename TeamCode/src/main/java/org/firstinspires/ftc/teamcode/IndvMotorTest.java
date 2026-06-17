package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "SingleMotorTest", group = "TeleOp")
public class IndvMotorTest extends OpMode {

    private DcMotorEx leftFrontDrive  = null;
    private DcMotorEx leftBackDrive   = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive  = null;

    // Index of currently selected motor: 0=LF, 1=RF, 2=LB, 3=RB
    private int selectedMotor = 0;
    private final String[] motorNames = {"Left Front", "Right Front", "Left Back", "Right Back",};

    private double testPower = 0.0;
    private static final double POWER_STEP = 0.05;

    // Edge detection so holding a button doesn't spam-trigger
    private boolean lastDpadUp    = false;
    private boolean lastDpadDown  = false;
    private boolean lastA         = false;

    @Override
    public void init() {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightbackdrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("A = run/stop selected motor");
        telemetry.addLine("Dpad Up/Down = select motor");
        telemetry.addLine("RB/LB = increase/decrease power");
        telemetry.update();
    }

    @Override
    public void loop() {

        // --- Motor selection (Dpad up/down, rising edge only) ---
        if (gamepad1.dpad_up && !lastDpadUp) {
            selectedMotor = (selectedMotor + 1) % 4;
            testPower = 0.0; // reset power when switching motors for safety
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            selectedMotor = (selectedMotor + 3) % 4; // -1 wrapped
            testPower = 0.0;
        }
        lastDpadUp   = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        // --- Power adjustment ---
        if (gamepad1.right_bumper) {
            testPower = Math.min(1.0, testPower + POWER_STEP);
        } else if (gamepad1.left_bumper) {
            testPower = Math.max(0.0, testPower - POWER_STEP);
        }

        // --- A button toggles run/stop instantly --
        boolean runMotor = gamepad1.a;

        // --- B button is emergency stop for everything ---
        if (gamepad1.b) {
            testPower = 0.0;
        }

        double appliedPower = runMotor ? testPower : 0.0;

        // Ensure only the selected motor receives power, all others are zero
        double lfPower = (selectedMotor == 0) ? appliedPower : 0.0;
        double rfPower = (selectedMotor == 1) ? appliedPower : 0.0;
        double lbPower = (selectedMotor == 2) ? appliedPower : 0.0;
        double rbPower = (selectedMotor == 3) ? appliedPower : 0.0;

        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lbPower);
        rightBackDrive.setPower(rbPower);

        // --- Telemetry ---
        telemetry.addData("Selected motor", motorNames[selectedMotor]);
        telemetry.addData("Test power",     String.format("%.2f", testPower));
        telemetry.addData("Running",        runMotor ? "YES (holding A)" : "NO");
        telemetry.addLine("");
        telemetry.addData("LF", String.format("power=%.2f  enc=%d  %.2fA",
                lfPower, leftFrontDrive.getCurrentPosition(),
                leftFrontDrive.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("RF", String.format("power=%.2f  enc=%d  %.2fA",
                rfPower, rightFrontDrive.getCurrentPosition(),
                rightFrontDrive.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("LB", String.format("power=%.2f  enc=%d  %.2fA",
                lbPower, leftBackDrive.getCurrentPosition(),
                leftBackDrive.getCurrent(CurrentUnit.AMPS)));
        telemetry.addData("RB", String.format("power=%.2f  enc=%d  %.2fA",
                rbPower, rightBackDrive.getCurrentPosition(),
                rightBackDrive.getCurrent(CurrentUnit.AMPS)));
        telemetry.update();
    }
}