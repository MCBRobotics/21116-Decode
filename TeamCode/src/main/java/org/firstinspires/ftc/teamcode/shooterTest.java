package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "shooterTest", group = "TeleOp")
public class shooterTest extends OpMode {
    private Servo leftServo;
    private DcMotor shooter;
    private Servo rightServo;
    @Override
    public void init(){
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        shooter = hardwareMap.get(DcMotor.class, "Shooter");
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }
    public void loop(){
        if (gamepad1.dpad_up){
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        if (gamepad1.dpad_down){
            leftServo.setPosition(0.0);
            rightServo.setPosition(0.0);
        if (gamepad1.a){
            shooter.setPower(1.0);
        }

        }
        }
        telemetry.addData("Left Servo", leftServo.getPosition());
        telemetry.addData("Right Servo", rightServo.getPosition());
        telemetry.update();
    }
}