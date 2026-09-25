package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = )
public class testIntake extends OpMode {
    private DcMotor Intake1;
    private DcMotor Intake2;

    @Override
    public void init() {
        Intake1 = hardwareMap.get(DcMotor.class, "intake1");
        Intake2 = hardwareMap.get(DcMotor.class, "intake2");
        Intake1.setDirection(DcMotor.Direction.FORWARD);
        Intake2.setDirection(DcMotor.Direction.FORWARD);
    }
    public void loop(){
        if (gamepad1.a){
            Intake1.setPower(1.0);
            Intake2.setPower(1.0);
        }else{
            Intake1.setPower(0.0);
            Intake2.setPower(0.0);
        }
    }
}
