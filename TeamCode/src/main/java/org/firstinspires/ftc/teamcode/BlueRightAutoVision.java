package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

@Autonomous(name="Blue Right Auto Vision", group = "Centerstage Autonomous", preselectTeleOp = "RobotController")
public class BlueRightAutoVision extends LinearOpMode {
    //Sets up motors and variables
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    Servo leftGripper;
    Servo rightGripper;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Pose2d selectPose = new Pose2d(0,0,0);
    double selectTurn = 0;
    double boardOffset = 0, zeOffset = 0;
    boolean armUp = false;
    int slidePos;
    double rampPos = 0.501;
    Servo arm;
    double slidePosError;
    double slidePower;
    long waitTime;
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

        leftGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
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

        //Sets drone launcher
        droneLauncher = hardwareMap.servo.get("droneLaunchServo");
        droneLauncher.setPosition(Constants.DRONE_START_POSITION);

        //Sets up dead wheels
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-38.25, 63.75, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Constants.blueAuto = false;

        waitForStart();

        //Move into spike mark area
        Trajectory approachSpikeMarks = drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(new Pose2d(-35, 40.25, startPose.getHeading()), startPose.getHeading())
                .lineToLinearHeading(new Pose2d(-35, 30.25, startPose.getHeading()))
                .build();
        drive.followTrajectory(approachSpikeMarks);

        TrajectorySequence placePurplePixel;

        //Detect prop
        if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
            selectPose = new Pose2d(-38.5, 31.25, Math.toRadians(180));
            selectTurn = -90;
            boardOffset = -6;
            zeOffset = 0;
            waitTime = 8;
            desiredTagId = 3;
            placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    //Turn to proper spike mark
                    .turn(Math.toRadians(selectTurn))
                    //Drive up to proper spike mark
                    .lineToLinearHeading(selectPose)
                    //Drop purple pixel
                    .addDisplacementMarker(() -> {
                        leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                    })
                    .back(5)
                    .turn(Math.toRadians(90))
                    .lineToLinearHeading(new Pose2d(-37, 11.5, Math.toRadians(270)))
                    .turn(Math.toRadians(90))
                    .build();
        } else if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
            selectPose = new Pose2d(-32.5, 31.25,0);
            selectTurn = 90;
            boardOffset = 6;
            zeOffset = 0;
            waitTime = 8;
            desiredTagId = 1;
            placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    //Turn to proper spike mark
                    .turn(Math.toRadians(selectTurn))
                    //Drive up to proper spike mark
                    .lineToLinearHeading(selectPose)
                    //Drop purple pixel
                    .addDisplacementMarker(() -> {
                        leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                    })
                    .back(7)
                    .turn(Math.toRadians(-90))
                    .lineToLinearHeading(new Pose2d(-38, 11.5, Math.toRadians(270)))
                    .turn(Math.toRadians(90))
                    .build();
        } else {
            selectPose = new Pose2d(-35.5, 31.5, Math.toRadians(90));
            selectTurn = 180;
            boardOffset = 0;
            zeOffset = 0;
            waitTime = 10;
            desiredTagId = 2;
            placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                    //Turn to proper spike mark
                    .turn(Math.toRadians(selectTurn))
                    //Drive up to proper spike mark
                    .lineToLinearHeading(new Pose2d(-40, 16, Math.toRadians(90)))
                    .addDisplacementMarker(() -> {
                        leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                    })
                    .back(8)
                    .turn(Math.toRadians(-90))
                    .build();
        }

        drive.followTrajectorySequence(placePurplePixel);

        //Waits for teammate to finish
        sleep(waitTime * 1000);

        TrajectorySequence driveToCenterOfField = drive.trajectorySequenceBuilder(placePurplePixel.end())
                //Drive towards center of field
                .lineToLinearHeading(new Pose2d(15,9,0))
                //Drive to board
                .splineToLinearHeading(new Pose2d(36,35.5 + boardOffset,0), Math.toRadians(90))
                .build();

        drive.followTrajectorySequence(driveToCenterOfField);

        //Raise slide and arm
        slidePos = 550;
        armUp = true;
        while ((slideMotor1.getCurrentPosition() < slidePos - 30 || ArmRamp.rampPos > Constants.ARM_UP_POS + 0.001) && opModeIsActive()) {
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

        //Variables needed for camera
        double za = 0;
        double ya = 0;
        double pa = 0;
        double ze = 0;
        double ye = 0;
        double pe = 0;
        double zrc = -0.75;
        double yrc = 4;
        long end = System.currentTimeMillis() + 1500;

        while(System.currentTimeMillis() < end)
        {
            List<AprilTagDetection> currentDetections = tagProcessor.getDetections();
            telemetry.addData("April tags detected", currentDetections.size());
            //Loops for each april tag detection
            for(AprilTagDetection detection : currentDetections)
            {
                if(detection.id == desiredTagId && detection.metadata != null)
                {
                    //Camera detection stuff
                    double zrcp = detection.ftcPose.z + zrc;
                    double yrcp = detection.ftcPose.y + yrc;
                    double prcp = detection.ftcPose.pitch;

                    //Convert camera measurements to robot
                    //https://en.wikipedia.org/wiki/Rotation_of_axes_in_two_dimensions
                    za = zrcp * Math.cos(-Math.toRadians(prcp)) + yrcp * Math.sin(-Math.toRadians(prcp));
                    ya = -zrcp * Math.sin(-Math.toRadians(prcp)) + yrcp * Math.cos(-Math.toRadians(prcp));
                    pa = -detection.ftcPose.pitch;

                    telemetry.addData("Tag ID", detection.id);

                    //Add some offsets
                    ze = -1.25 + zrc - za - zeOffset;
                    ye = 19.2 + yrc - ya;
                    pe = 0 - pa;

                    telemetry.addData("ze", ze);
                    telemetry.addData("ye", ye);
                    telemetry.addData("pe", pe);

                    break;  //For loop
                }
            }
            telemetry.update();
            if(ze != 0){
                break;  //While loop
            }
        }

        //Drives forward to board
        Trajectory driveToBoard = drive.trajectoryBuilder(driveToCenterOfField.end())
                .lineToLinearHeading(new Pose2d(49 - ye, 35.5 + boardOffset - ze, Math.toRadians(0 + pe)))
                .build();
        drive.followTrajectory(driveToBoard);
        //Opens gripper
        rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);
        //Waits for pixel to drop
        sleep(600);
        //Drives back from board
        Trajectory driveBackFromBoard = drive.trajectoryBuilder(driveToBoard.end())
                .back(5)
                .build();
        drive.followTrajectory(driveBackFromBoard);

        Constants.autoEndPose = driveBackFromBoard.end();

        //Lower slide and arm
        slidePos = 0;
        armUp = false;
        while ((slideMotor1.getCurrentPosition() > slidePos + 50 || ArmRamp.rampPos < Constants.ARM_DOWN_POS - 0.001) && opModeIsActive()) {
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

        sleep(30000);
    }
}
