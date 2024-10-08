package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.fininestatemachines.LeftGripper;
import org.firstinspires.ftc.teamcode.fininestatemachines.RightGripper;
import org.firstinspires.ftc.teamcode.functions.ArmRamp;
import org.firstinspires.ftc.teamcode.functions.Constants;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="RobotController2")
public class FiniteStateMachineRobotController extends LinearOpMode{
    // Initializing motors and variables
    DcMotor frontLeftMotor;
    DcMotor rearLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearRightMotor;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Servo arm;
    Servo droneLauncher;
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;
    RightGripper rightGripper;
    LeftGripper leftGripper;

    // Variables used for auto turn
    double turnError = 0;
    double turnSpeed = 0;
    double turnSetpoint = 0;
    double turnFeedback = 0;
    double turnOffset = 0;
    boolean fieldOriented = true;
    boolean fieldOrientedToggle = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        double y = 0;
        double x = 0;

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);
        //Setting up motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        rearLeftMotor = hardwareMap.dcMotor.get("rearLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        rearRightMotor = hardwareMap.dcMotor.get("rearRightMotor");

        rearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        slideMotor1 = hardwareMap.dcMotor.get("leftSlideMotor");
        slideMotor2 = hardwareMap.dcMotor.get("rightSlideMotor");

        slideMotor2.setDirection(DcMotor.Direction.REVERSE);

        slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Setting up servos
        arm = hardwareMap.servo.get("armServo");
        droneLauncher = hardwareMap.servo.get("droneLaunchServo");

        leftSensor = hardwareMap.get(DistanceSensor.class, "checkLeft");
        rightSensor = hardwareMap.get(DistanceSensor.class, "checkRight");
        //Setting up dead wheels
        myLocalizer.setPoseEstimate(Constants.autoEndPose);
        if(Constants.blueAuto){
            turnSetpoint = Math.toDegrees(Constants.autoEndPose.getHeading()) + 90;
        }
        else{
            turnSetpoint = Math.toDegrees(Constants.autoEndPose.getHeading()) - 90;
        }

        waitForStart();
        //Set start servo positions
        droneLauncher.setPosition(Constants.DRONE_START_POSITION);
        leftGripper.OpenGripper();
        rightGripper.OpenGripper();

        if (isStopRequested()) return;

        double slowingDown = 4;
        double slowDownTurning = 4;

        while (opModeIsActive()) {

            myLocalizer.update();
            Pose2d myPose = myLocalizer.getPoseEstimate();
            //Launch drone if up button is pressed
            if ((gamepad1.dpad_up)){
                droneLauncher.setPosition(Constants.DRONE_RELEASE_POSITION);
            }
            //Slows down robot move speed for better control
            if(Math.abs(gamepad1.right_stick_x) > 0.3 || Math.abs(gamepad1.right_stick_y) > 0.3 || gamepad1.right_stick_button || slidePos > 100){
                slowingDown = 4;
                slowDownTurning = 3;
            }
            else{
                slowingDown = 1;
                slowDownTurning = 1;
            }
            //Gets gamepad input
            y = (-gamepad1.left_stick_y / slowingDown);
            x = (gamepad1.left_stick_x / slowingDown);
            turnOffset = (gamepad1.left_trigger - gamepad1.right_trigger) * (2.5 / slowDownTurning);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //Calls attachment movement(slides, arm, and grippers)
            AttachmentController();
            //Resets robot orientation
            if (gamepad1.back){
                myLocalizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));
                turnSetpoint = 0;
                Constants.blueAuto = false;
            }
            else{
                //Switches between field and robot orientation
                if(gamepad1.start && !fieldOrientedToggle && fieldOriented){
                    fieldOriented = false;
                    fieldOrientedToggle = true;
                }
                else if(gamepad1.start && !fieldOrientedToggle && !fieldOriented){
                    fieldOriented = true;
                    fieldOrientedToggle = true;
                }
                else if(!gamepad1.start) {
                    fieldOrientedToggle = false;
                }

                if(fieldOriented){
                    //Reverses robot direction when on blue side
                    if(Constants.blueAuto){
                        turnFeedback = myPose.getHeading() + Math.toRadians(90);
                    }
                    else{
                        turnFeedback = myPose.getHeading() - Math.toRadians(90);
                    }
                    //Auto-turn code
                    Direction();
                    GyroTurn();
                }
                else {
                    //Robot oriented code
                    turnOffset /= 4;
                    turnSpeed = turnOffset;
                    turnFeedback = 0;
                    turnSetpoint = myPose.getHeading();
                }
                //Field oriented algorithms
                double rotX = x * Math.cos(-turnFeedback) - y * Math.sin(-turnFeedback);
                double rotY = x * Math.sin(-turnFeedback) + y * Math.cos(-turnFeedback);
                rotX = rotX * 1.1;
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnSpeed), 1);

                double frontLeftPower = (rotY + rotX - turnSpeed) / denominator;
                double rearLeftPower = (rotY - rotX - turnSpeed) / denominator;
                double frontRightPower = (rotY - rotX + turnSpeed) / denominator;
                double rearRightPower = (rotY + rotX + turnSpeed) / denominator;
                //Stops motors from running if motor power is less then %5
                if((Math.abs(frontLeftPower) < 0.05) & (Math.abs(rearLeftPower) < 0.05) & (Math.abs(frontRightPower) < 0.05) & (Math.abs(rearRightPower) < 0.05))
                {
                    frontLeftMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    rearRightMotor.setPower(0);
                }
                else
                {
                    //Sets motor power
                    frontLeftMotor.setPower(frontLeftPower);
                    rearLeftMotor.setPower(rearLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    rearRightMotor.setPower(rearRightPower);
                }
                //Telemetry for debugging
                telemetry.addData("Field Oriented?", fieldOriented);
                telemetry.addLine();
                telemetry.addData("Turn Setpoint", turnSetpoint);
                telemetry.addData("Turn Heading", Math.toDegrees(turnFeedback));
                telemetry.addData("Turn Error", turnError);
                telemetry.addData("Turn Speed", turnSpeed);
                telemetry.addLine();
                telemetry.addData("Left Distance Sensor", leftSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Right Distance Sensor", rightSensor.getDistance(DistanceUnit.CM));

                telemetry.update();
            }
        }
    }
    //Variables for attachment movement
    int slidePos = 0;
    boolean armPos = false;
    boolean slideResetOneShot = false;
    double slidePosError, slidePower = 0, lastSlidePower = 0, slidePowerIncr = 0.1;
    public void AttachmentController()
    {
        //Sets slide and arm position when button is pressed
        if(gamepad2.a) {
            slidePos = Constants.SLIDE_LOW_POS;
            armPos = true;
        } else if (gamepad2.b) {
            slidePos = Constants.SLIDE_MEDIUM_POS;
            armPos = true;
        } else if (gamepad2.y) {
            slidePos = Constants.SLIDE_HIGH_POS;
            armPos = true;
        } else if (gamepad2.x) {
            slidePos = Constants.SLIDE_STACK_POS;
            armPos = false;
        }
        //Resets slide and arm positions
        if(gamepad2.dpad_down)
        {
            slidePos = 0;
            armPos = false;
            //Resets gripper position
            leftGripper.OpenGripper();
            rightGripper.OpenGripper();
        }
        //Manual slide movement
        slidePos += -gamepad2.left_stick_y * 55;
        //Clamps slide position so motors don't stall
        slidePos = Math.min(Math.max(slidePos, 0), 3070);
        //Moves slide down if slide position is set incorrectly
        if(gamepad2.back){
            slideResetOneShot = true;
            slideMotor1.setPower(-0.3);
            slideMotor2.setPower(-0.3);
        }
        else if(slideResetOneShot){
            //Resets slide encoder
            slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideResetOneShot = false;
        }
        else if(slideMotor1.getCurrentPosition() < 50 && slidePos < 5){
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
            slideMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        else if(gamepad2.start) {
            //Increases slide power when hanging
            slidePos = 1000;

            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 5 / 500;
            slidePower = Math.min(Math.max(slidePower, -0.75), 0.75);

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);
        }
        else {
            //Moves slides
            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 1.25 / 500;
            slidePower = Math.min(Math.max(slidePower, -0.7), 0.85);

            if(slidePower - lastSlidePower > slidePowerIncr)
            {
                slidePower = lastSlidePower + slidePowerIncr;
            }

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);
            telemetry.addData("slidePosError", slidePosError);
        }
        //More telemetry for debugging
        telemetry.addData("slideMotor1 Power", slideMotor1.getPower());
        telemetry.addData("slidePosition", slidePos);
        telemetry.addData("Current Slide Position", slideMotor1.getCurrentPosition());

        //Gripper controls
        leftGripper.LeftGripperMaster(gamepad2.left_bumper, gamepad2.left_trigger > 0.5);
        rightGripper.RightGripperMaster(gamepad2.right_bumper, gamepad2.right_trigger > 0.5);

        //Sets arm position
        if(armPos || slideMotor1.getCurrentPosition() < 550)
        {
            arm.setPosition(ArmRamp.Ramp(Constants.ARM_DOWN_POS, Constants.ARM_UP_POS, armPos));
        }
        telemetry.addData("Arm pos", armPos);

        lastSlidePower = slideMotor1.getPower();
    }

    public void GyroTurn()
    {
        //Calculating auto turn motor powers
        turnError = turnSetpoint - Math.toDegrees(turnFeedback);
        if(turnError > 180){
            turnError = turnError - 360;
        }
        else if(turnError < -180){
            turnError = turnError + 360;
        }

        // Rotation sensitivity
        turnSpeed = 0.04 * turnError;
        // Keeps turning speed in a range
        turnSpeed = Math.min(Math.max(turnSpeed, -0.4), 0.4);
    }

    public void Direction()
    {
        //Setting auto-turn target position
        if(gamepad1.x){
            turnSetpoint = 90;
        }
        else if(gamepad1.y){
            turnSetpoint = 0;
        }
        else if(gamepad1.b){
            turnSetpoint = 270;
        }
        else if(gamepad1.a){
            turnSetpoint = 180;
        }
        else{
            turnSetpoint += turnOffset;
        }
        //Keeps auto turn between 0 to 360 degrees
        if(turnSetpoint > 360){
            turnSetpoint = turnSetpoint - 360;
        }
        else if(turnSetpoint < 0){
            turnSetpoint = turnSetpoint + 360;
        }
    }
}
