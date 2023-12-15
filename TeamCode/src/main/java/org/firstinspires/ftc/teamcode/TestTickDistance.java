package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="TestTickDistance")
public class TestTickDistance extends LinearOpMode {

    DcMotor slideMotor;
    DcMotor slideMotor2;
    @Override
    public void runOpMode() throws InterruptedException
    {
        slideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Slide Ticks", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Ticks", slideMotor2.getCurrentPosition());
            telemetry.update();
        }
    }
}
