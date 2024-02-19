package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    double gripper1 = 0.5;
    double gripper2 = 0.5;
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
        arm = hardwareMap.servo.get("armServo");
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");
        droneLauncher = hardwareMap.servo.get("droneLaunchServo");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drone += gamepad1.left_stick_y / 500;
            armPosition += gamepad1.right_stick_y / 500;
            gripper1 += gamepad2.left_stick_y / 500;
            gripper2 += gamepad2.right_stick_y / 500;

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



            telemetry.update();

        }
    }
}
