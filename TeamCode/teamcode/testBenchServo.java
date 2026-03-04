package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class testBenchServo {
    private Servo servoPos;

    public void init(HardwareMap hwMap){
        servoPos = hwMap.get(Servo.class, "Servo1");
    }

    public void setServoPos(double angle) {
        servoPos.setPosition(angle);
    }
}
