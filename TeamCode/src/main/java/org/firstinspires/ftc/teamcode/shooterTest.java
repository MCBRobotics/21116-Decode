    package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.CRServo;

    @TeleOp(name = "shooterTest", group = "TeleOp")
    public class shooterTest extends OpMode {
        private CRServo leftServo;
        private DcMotor shooter;
        private CRServo rightServo;
        private CRServo midServo;
        private Servo hoodServo;
        @Override
        public void init(){
            rightServo = hardwareMap.get(CRServo.class, "rightServo");
            leftServo = hardwareMap.get(CRServo.class, "leftServo");
            midServo = hardwareMap.get(CRServo.class, "midServo");
            hoodServo = hardwareMap.get(Servo.class, "hoodServo");
            shooter = hardwareMap.get(DcMotor.class, "Shooter");
            shooter.setDirection(DcMotor.Direction.FORWARD);
            rightServo.setDirection(CRServo.Direction.FORWARD);
            leftServo.setDirection(CRServo.Direction.FORWARD);
            midServo.setDirection(CRServo.Direction.FORWARD);
            hoodServo.setPosition(0.5);

        }
        public void loop(){
            if (gamepad1.dpad_up) {
                hoodServo.setPosition(1.0);
            } else if (gamepad1.dpad_down) {
                hoodServo.setPosition(0.0);
            }
            if (gamepad1.dpad_left) {
                hoodServo.setPosition(0.5);
            }
            if (gamepad1.a) {
                shooter.setPower(1.0);
            } else{
                shooter.setPower(0.0);
            }
            //After initial test add steps for hoodServo
            //May need to adjust power depending on test results
            if (gamepad1.left_bumper){
                rightServo.setPower(-1.0);
                leftServo.setPower(-1.0);
                midServo.setPower(-1.0);
            } else{
                rightServo.setPower(0.0);
                leftServo.setPower(0.0);
                midServo.setPower(0.0);
            }
            if (gamepad1.right_bumper){
                rightServo.setPower(1.0);
                leftServo.setPower(1.0);
                midServo.setPower(1.0);
            }else {
                rightServo.setPower(0.0);
                leftServo.setPower(0.0);
                midServo.setPower(0.0);
            }
            telemetry.addData("Hood Servo", hoodServo.getPosition());
            double power = shooter.getPower();
            boolean readyToFire = Math.abs(power - 1.0) < 0.01;
            telemetry.addData("Shooter Power", power);
            telemetry.addData("Status", readyToFire ? "READY TO FIRE" : "NOT READY");
            telemetry.update();
            }
        }
