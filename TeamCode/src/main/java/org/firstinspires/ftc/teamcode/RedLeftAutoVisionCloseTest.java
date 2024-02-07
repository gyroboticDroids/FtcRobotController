package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Red Left Auto Vision Test", group = "Centerstage Autonomous", preselectTeleOp = "RobotController")
public class RedLeftAutoVisionCloseTest extends LinearOpMode {
    //Sets up motors and variables
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    Servo leftGripper;
    Servo rightGripper;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Pose2d selectPose = new Pose2d(0,0,0);
    double selectTurn = 0;
    double boardOffset = 0;
    boolean armUp = false;
    int slidePos;
    double rampPos = 0.501;
    Servo arm;
    double slidePosError;
    double slidePower;
    double waitTime;
    int desiredTagId;
    Servo droneLauncher;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Sets up camera
        AprilTagProcessor tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), tagProcessor);

        //Sets sensors
        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");
        //Sets grippers and other servos
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");

        leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
        rightGripper.setPosition(Constants.GRIPPER_RIGHT_CLOSE_POSITION);
        arm = hardwareMap.servo.get("armServo");
        //Sets slide motors
        slideMotor1 = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");

        slideMotor2.setDirection(DcMotor.Direction.REVERSE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        droneLauncher = hardwareMap.servo.get("droneLaunchServo");
        droneLauncher.setPosition(Constants.DRONE_START_POSITION);
        //Sets up dead wheels
        boardOffset = 0;
        desiredTagId = 5;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(36,-32.5 + boardOffset,0);
        drive.setPoseEstimate(startPose);

        waitForStart();

//        TrajectorySequence approachSpikeMarks = drive.trajectorySequenceBuilder(startPose)
//                //Move into spike mark area
//                .splineToLinearHeading(new Pose2d(-35, -30.25, startPose.getHeading()), startPose.getHeading())
//                //Determine where prop is located
//                .addDisplacementMarker(() -> {
//                    if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
//                        selectPose = new Pose2d(-38.5, -31.25, Math.toRadians(180));
//                        selectTurn = 90;
//                        boardOffset = 6;
//                        waitTime = 3;
//                        desiredTagId = 4;
//                    } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
//                        selectPose = new Pose2d(-32.5, -31.25,0);
//                        selectTurn = -90;
//                        boardOffset = -6;
//                        waitTime = 0;
//                        desiredTagId = 6;
//                    } else {
//                        selectPose = new Pose2d(-35.5, -31.5, Math.toRadians(90));
//                        selectTurn = 0;
//                        boardOffset = 0;
//                        waitTime = 1.75;
//                        desiredTagId = 5;
//                    }
//                })
//                .build();
//        drive.followTrajectorySequence(approachSpikeMarks);
//
//        TrajectorySequence placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
//                //Turn to proper spike mark
//                .turn(Math.toRadians(selectTurn))
//                //Drive up to proper spike mark
//                .lineToLinearHeading(selectPose)
//                //Drop purple pixel
//                .addDisplacementMarker(() -> {
//                    leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
//                })
//                //Back away from purple pixel
//                .back(4.5)
//                //Move back to safe turning position
//                .lineToLinearHeading(new Pose2d(-37, -48, selectPose.getHeading()))
//                //Turn towards back wall
//                .turn(Math.toRadians(179.999)-selectPose.getHeading())
//                //Drive towards back wall
//                .lineToLinearHeading(new Pose2d(-58,-48, Math.toRadians(180)))
//                //Turn towards opposing side
//                .turn(Math.toRadians(-90))
//                //Drive towards opposing side
//                .lineToLinearHeading(new Pose2d(-58,-9, Math.toRadians(90)))
//                //Turn towards center of field
//                .turn(Math.toRadians(-90))
//                //Drive towards center of field
//                .lineToLinearHeading(new Pose2d(15,-9,0))
//                //Wait for teammate to finish
//                .waitSeconds(waitTime)
//                //Drive to board
//                .splineToLinearHeading(new Pose2d(36,-32.5 + boardOffset,0), Math.toRadians(-90))
//                .build();
//
//        drive.followTrajectorySequence(placePurplePixel);

        //Raise slide and arm
        slidePos = 550;
        armUp = true;
        while ((slideMotor1.getCurrentPosition() < slidePos - 30 || rampPos < Constants.ARM_UP_POS - 0.001) && opModeIsActive()) {
            arm.setPosition(Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armUp));
            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 3 / 500;
            slidePower = Math.min(Math.max(slidePower, -0.6), 0.75);

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);
            telemetry.addData("Slide Pos", slideMotor1.getCurrentPosition());
            telemetry.addData("Arm Pos", arm.getPosition());
            telemetry.update();
        }
        slideMotor1.setPower(0);
        slideMotor2.setPower(0);

        double za = 0;
        double ya = 0;
        double pa = 0;
        double ze = 0;
        double ye = 0;
        double pe = 0;
        sleep(2000);

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("April tags detected", currentDetections.size());
        for(AprilTagDetection detection : currentDetections)
        {
            if(detection.id == desiredTagId && detection.metadata != null)
            {
                za = detection.ftcPose.z * Math.cos(-Math.toRadians(detection.ftcPose.pitch)) + detection.ftcPose.y * Math.sin(-Math.toRadians(detection.ftcPose.pitch));
                ya = -detection.ftcPose.z * Math.sin(-Math.toRadians(detection.ftcPose.pitch)) + detection.ftcPose.y * Math.cos(-Math.toRadians(detection.ftcPose.pitch));
                pa = -detection.ftcPose.pitch;

                telemetry.addData("Tag ID", detection.id);

                ze = -1.5 - za;
                ye = 19.2 - ya;
                pe = 0 - pa;

                telemetry.addData("ze", ze);
                telemetry.addData("ye", ye);
                telemetry.addData("pe", pe);

                break;
            }
        }
        telemetry.update();
        sleep(5000);

        TrajectorySequence dropOnBoard = drive.trajectorySequenceBuilder(startPose)
                //Drive to place yellow pixel
                //TODO: Add camera here
                .lineToLinearHeading(new Pose2d(49 - ye, -31 + boardOffset - ze, Math.toRadians(0 + pe)))
                //.lineToConstantHeading(new Vector2d(49 - ye, -31 + boardOffset - ze))
                //Drop yellow pixel
                .addDisplacementMarker(() -> {
                    rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);
                })
                //Wait for gripper to open
                .waitSeconds(0.1)
                //Back up to release pixel if trapped against backdrop
                .back(5)
                .build();
        drive.followTrajectorySequence(dropOnBoard);

        //Lower slide and arm
        slidePos = 0;
        armUp = false;
        while ((slideMotor1.getCurrentPosition() > slidePos + 50 || rampPos > Constants.ARM_DOWN_POS + 0.001) && opModeIsActive()) {
            arm.setPosition(Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armUp));
            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 3 / 500;
            slidePower = Math.min(Math.max(slidePower, -0.6), 0.75);

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);

            telemetry.addData("Slide Pos", slideMotor1.getCurrentPosition());
            telemetry.addData("Arm Pos", arm.getPosition());
            telemetry.update();
        }
        slideMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slideMotor1.setPower(0);
        slideMotor2.setPower(0);

        Constants.autoEndPose = dropOnBoard.end();
        Constants.blueAuto = false;
    }
    public double Ramp(double firstPos, double secondPos, boolean selectPos)
    {
        //Slows down arm servo so it does not break itself
        if(selectPos){
            rampPos += 0.01;
        }
        else{
            rampPos -= 0.01;
        }
        rampPos = Math.min(Math.max(rampPos, firstPos), secondPos);

        return rampPos;
    }
}
