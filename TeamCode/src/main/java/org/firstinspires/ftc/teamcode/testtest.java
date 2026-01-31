 package org.firstinspires.ftc.teamcode;

 import androidx.annotation.NonNull;

// import com.acmerobotics.dashboard.config.Config;
// import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
// import com.acmerobotics.roadrunner.Action;
// import com.acmerobotics.roadrunner.Pose2d;
// import com.acmerobotics.roadrunner.SequentialAction;
// import com.acmerobotics.roadrunner.Vector2d;
// import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "FirstAuto", group = "Autonomous") 
public class testtest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor IntakeM = null;

    public class Shooter  {
            /*public class startIntake() implements Action {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    IntakeM.setPower(1);
                    return false;
                }

            }
            public Action StartIntake() {
                return new startIntake();
            }*/

    }

    public void runOpMode() {

        waitForStart();
        Shooter shooter = new Shooter();

        if (isStopRequested()) return;
        //Actions.runBlocking(shooter.StartIntake());


    }

}


