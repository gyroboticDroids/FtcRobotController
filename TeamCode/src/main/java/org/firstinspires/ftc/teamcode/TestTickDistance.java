package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestTickDistance")
public class TestTickDistance extends LinearOpMode {

    DcMotor slideMotor;
    DcMotor slideMotor2;
    Servo arm;
    Servo leftGripper;
    Servo rightGripper;
    Servo droneLauncher;
    DcMotor frontLeftMotor;
    DcMotor rearLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearRightMotor;
    double armPosition = 0.501;
    double drone = 0.696;
    double gripper1;
    double gripper2;
    @Override
    public void runOpMode() throws InterruptedException
    {
        slideMotor = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.speak("Hi. Please press play to run the program", "Latin", "Mexico");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");
        slideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor2.setDirection(DcMotor.Direction.REVERSE);
        arm = hardwareMap.servo.get("armServo");
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");
        droneLauncher = hardwareMap.servo.get("droneLaunchServo");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drone += gamepad1.left_stick_y / 100;
            armPosition += gamepad1.right_stick_y / 100;
            gripper1 += gamepad2.left_stick_y / 100;
            gripper2 += gamepad2.right_stick_y / 100;

            drone += gamepad1.left_stick_y / 100;
            armPosition += gamepad1.right_stick_y / 100;

            arm.setPosition(armPosition);
            droneLauncher.setPosition(drone);

            leftGripper.setPosition(gripper1);
            rightGripper.setPosition(gripper2);

            telemetry.addData("Slide Ticks", slideMotor.getCurrentPosition());
            telemetry.addData("Slide Ticks", slideMotor2.getCurrentPosition());

            telemetry.addData("Left Gripper Position", leftGripper.getPosition());
            telemetry.addData("Right Gripper Position", rightGripper.getPosition());

            telemetry.addData("Drone Servo Position", droneLauncher.getPosition());
            telemetry.addData("Arm Position", arm.getPosition());
            telemetry.addData("Left Gripper Position", leftGripper.getPosition());
            telemetry.addData("Right Gripper Position", rightGripper.getPosition());

            telemetry.addData("Pod 1", frontLeftMotor.getCurrentPosition());
            telemetry.addData("Pod 2", rearLeftMotor.getCurrentPosition());
            telemetry.addData("Pod 3", frontRightMotor.getCurrentPosition());
            telemetry.addData("Pod 4", rearRightMotor.getCurrentPosition());

            telemetry.addData("Pod 3", frontRightMotor.getCurrentPosition());
            telemetry.addData("Pod 4", rearRightMotor.getCurrentPosition());


            telemetry.update();

        }
    }
}
