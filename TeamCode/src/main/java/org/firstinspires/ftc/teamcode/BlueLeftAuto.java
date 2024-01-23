package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Blue Left Auto", group = "Centerstage Autonomous", preselectTeleOp = "RobotController")
public class BlueLeftAuto extends LinearOpMode {
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

    Servo droneLauncher;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //Sets up sensors
        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");
        //Sets up servos
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");

        leftGripper.setPosition(Constants.GRIPPER_LEFT_CLOSE_POSITION);
        rightGripper.setPosition(Constants.GRIPPER_RIGHT_CLOSE_POSITION);
        arm = hardwareMap.servo.get("armServo");
        //Sets up slides
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(14.5, 63.75, Math.toRadians(-90));
        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence approachSpikeMarks = drive.trajectorySequenceBuilder(startPose)
                //Move into spike mark area
                .splineToLinearHeading(new Pose2d(11.75, 30.25, startPose.getHeading()), startPose.getHeading())
                //Determine where prop is located
                .addDisplacementMarker(() -> {
                    if (leftSensor.getDistance(DistanceUnit.INCH) < 5) {
                        selectPose = new Pose2d(15.75, 31.25, Math.toRadians(0));
                        selectTurn = 90;
                        boardOffset = 4.25;
                    } else if (rightSensor.getDistance(DistanceUnit.INCH) < 5) {
                        selectPose = new Pose2d(7.75, 31.25, Math.toRadians(180));
                        selectTurn = -90;
                        boardOffset = -4.25;
                    } else {
                        selectPose = new Pose2d(11.75, 31.5, Math.toRadians(-90));
                        selectTurn = 0;
                        boardOffset = 1.5;
                    }
                })
                .build();
        drive.followTrajectorySequence(approachSpikeMarks);

        TrajectorySequence placePurplePixel = drive.trajectorySequenceBuilder(approachSpikeMarks.end())
                //Turn to proper spike mark
                .turn(Math.toRadians(selectTurn))
                //Drive up to proper spike mark
                .lineToLinearHeading(selectPose)
                //Drop purple pixel
                .addDisplacementMarker(() -> {
                    leftGripper.setPosition(Constants.GRIPPER_LEFT_OPEN_POSITION);
                })
                //Back away from purple pixel
                .back(4.5)
                //Move back to safe turning position
                .lineToLinearHeading(new Pose2d(13.75, 50, selectPose.getHeading()))
                //Turn towards back wall
                .turn(Math.toRadians(0.001)-selectPose.getHeading())
                //Drives to board
                .splineToLinearHeading(new Pose2d(39,37 + boardOffset,0),Math.toRadians(-90))
                .build();

        drive.followTrajectorySequence(placePurplePixel);

        //Raise slide and arm
        slidePos = 320;
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

        TrajectorySequence dropOnBoard = drive.trajectorySequenceBuilder(placePurplePixel.end())
                //Drive to place yellow pixel
                .forward(49.5 - placePurplePixel.end().getX())
                //Drop yellow pixel
                .addDisplacementMarker(() -> {
                    rightGripper.setPosition(Constants.GRIPPER_RIGHT_OPEN_POSITION);
                })
                //Wait for gripper to open
                .waitSeconds(0.4)
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

        TrajectorySequence moveToCorner = drive.trajectorySequenceBuilder(dropOnBoard.end())
                //Moves to back right corner
                .lineToConstantHeading(new Vector2d(44.5, 60))
                //Scoots forward to get out of the way
                .forward(10)
                .build();
        drive.followTrajectorySequence(moveToCorner);

        Constants.autoEndPose = moveToCorner.end();
        Constants.blueAuto = true;
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
