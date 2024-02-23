package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="Old Red Left Auto Vision Close Test", group = "Centerstage Autonomous", preselectTeleOp = "RobotController")
@Disabled
public class OldRedLeftAutoVisionCloseTest extends LinearOpMode {
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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        boardOffset = 0;
        desiredTagId = 5;
        Pose2d startPose = new Pose2d(36,-32.5 + boardOffset,0);
        //Pose2d startPose = new Pose2d(-38.25, -63.75, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        waitForStart();

//        Trajectory approachSpikeMarks = drive.trajectoryBuilder(startPose)
//                //Move into spike mark area
//                .splineToLinearHeading(new Pose2d(-35, -30.25, startPose.getHeading()), startPose.getHeading())
//                .build();
//        drive.followTrajectory(approachSpikeMarks);
//
//        //Determine where prop is located
//        if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
//            selectPose = new Pose2d(-38.5, -31.25, Math.toRadians(180));
//            selectTurn = 90;
//            boardOffset = 6;
//            waitTime = 3;
//            desiredTagId = 4;
//        } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
//            selectPose = new Pose2d(-32.5, -31.25,0);
//            selectTurn = -90;
//            boardOffset = -6;
//            waitTime = 0;
//            desiredTagId = 6;
//        } else {
//            selectPose = new Pose2d(-35.5, -31.5, Math.toRadians(90));
//            selectTurn = 0;
//            boardOffset = 0;
//            waitTime = 1.75;
//            desiredTagId = 5;
//        }
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
            arm.setPosition(ArmRamp.Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armUp));
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
        double zrc = -0.75;
        double yrc = 4;
        sleep(1000);

        List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
        telemetry.addData("April tags detected", currentDetections.size());
        for(AprilTagDetection detection : currentDetections)
        {
            if(detection.id == desiredTagId && detection.metadata != null)
            {
                double zrcp = detection.ftcPose.z + zrc;
                double yrcp = detection.ftcPose.y + yrc;
                double prcp = detection.ftcPose.pitch;

                za = zrcp * Math.cos(-Math.toRadians(prcp)) + yrcp * Math.sin(-Math.toRadians(prcp));
                ya = -zrcp * Math.sin(-Math.toRadians(prcp)) + yrcp * Math.cos(-Math.toRadians(prcp));
                pa = -detection.ftcPose.pitch;

                telemetry.addData("Tag ID", detection.id);

                ze = -1.25 + zrc - za;
                ye = 19.2 + yrc - ya;
                pe = 0 - pa;

                telemetry.addData("ze", ze);
                telemetry.addData("ye", ye);
                telemetry.addData("pe", pe);

                break;
            }
        }
        telemetry.update();
        sleep(5000);

        Trajectory driveToBoard = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(49 - ye, -31 + boardOffset - ze, Math.toRadians(0 + pe)))
                .build();
        drive.followTrajectory(driveToBoard);

        rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);

        sleep(1000);

        Trajectory driveBackFromBoard = drive.trajectoryBuilder(driveToBoard.end())
                .back(5)
                .build();
        drive.followTrajectory(driveBackFromBoard);

//        TrajectorySequence dropOnBoard = drive.trajectorySequenceBuilder(startPose)
//                //Drive to place yellow pixel
//                //TODO: Add camera here
//                .lineToLinearHeading(new Pose2d(49 - ye, -31 + boardOffset - ze, Math.toRadians(0 + pe)))
//                //.lineToConstantHeading(new Vector2d(49 - ye, -31 + boardOffset - ze))
//                //Drop yellow pixel
//                .addDisplacementMarker(() -> {
//                    rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);
//                })
//                //Wait for gripper to open
//                .waitSeconds(0.1)
//                //Back up to release pixel if trapped against backdrop
//                .back(5)
//                .build();
//        drive.followTrajectorySequence(dropOnBoard);

        //Lower slide and arm
        slidePos = 0;
        armUp = false;
        while ((slideMotor1.getCurrentPosition() > slidePos + 50 || rampPos > Constants.ARM_DOWN_POS + 0.001) && opModeIsActive()) {
            arm.setPosition(ArmRamp.Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armUp));
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

        Constants.autoEndPose = driveBackFromBoard.end();
        Constants.blueAuto = false;
    }
}
