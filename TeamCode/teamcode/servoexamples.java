package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class servoExamples extends OpMode {
    testBenchServo bench = new testBenchServo();
    @Override
    public void init(){
        bench.init();
    }
    @Override
    public void loop(){

    }
}
