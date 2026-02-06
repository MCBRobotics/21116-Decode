package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CR Servo Test", group = "Linear Opmode")
public class testBenchServo extends OpMode {

    private CRServo crServo;

    @Override
    public void init() {

        crServo = hardwareMap.get(CRServo.class, "crservo");

        // Make sure it stops at init
        crServo.setPower(0.0);
    }

    @Override
    public void loop() {

        if (gamepad1.y) {
            crServo.setPower(1.0); // Forward
        }
        else if (gamepad1.a) {
            crServo.setPower(-1.0); // Reverse
        }
        else if (gamepad1.square) {
            crServo.setPower(0.0); // Stop
        }
    }
}