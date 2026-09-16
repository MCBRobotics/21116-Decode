package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servotest", group = "TeleOp")
public class ServoTest extends OpMode {
    private Servo leftServo;
    private Servo rightServo;
    @Override
    public void init(){
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }
    public void loop(){
        if (gamepad1.a){
            leftServo.setPosition(1.0);
            rightServo.setPosition(1.0);
        if (gamepad1.b){
            leftServo.setPosition(0.0);
            rightServo.setPosition(0.0);
        }
        }
        telemetry.addData("Left Servo", leftServo.getPosition());
        telemetry.addData("Right Servo", rightServo.getPosition());
        telemetry.update();
    }
}