package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import  com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RobotController")
public class RobotController extends LinearOpMode{
    double error = 0;
    double rotationSpeed = 0;
    double setPoint = 0;
    double feedback = 0;
    double rx = 0;


    DcMotor frontLeftMotor;
    DcMotor rearLeftMotor;
    DcMotor frontRightMotor;
    DcMotor rearRightMotor;
    DcMotor slideMotor1;
    DcMotor slideMotor2;
    Servo arm;
    Servo leftGripper;
    Servo rightGripper;
    DistanceSensor leftSensor;
    DistanceSensor rightSensor;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException
    {
    //    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

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
        //slideMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //slideMotor1.setTargetPosition(0);
        //slideMotor2.setTargetPosition(0);

        //slideMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //slideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm = hardwareMap.servo.get("armServo");
        leftGripper = hardwareMap.servo.get("leftGripperServo");
        rightGripper = hardwareMap.servo.get("rightGripperServo");

        leftSensor = hardwareMap.get(DistanceSensor.class, "leftCheck");
        rightSensor = hardwareMap.get(DistanceSensor.class, "rightCheck");

        ConfigureG();

        waitForStart();

        if (isStopRequested()) return;

        double slowingDown = 4;

        while (opModeIsActive()) {

            if(gamepad1.b){
                slowingDown = 4;
            }
            else{
                slowingDown = 1.5;
            }

            double y = (-gamepad1.left_stick_y / slowingDown) + (gamepad1.right_stick_y / -2);
            double x = (gamepad1.left_stick_x / slowingDown) + (gamepad1.right_stick_x / 2);
            rx = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5;

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            AttachmentController();

            double botHeading;
       //     botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            botHeading = 0;

            if (gamepad1.back){
                imu.resetYaw();
                setPoint = 0;
            }
            else if((botHeading == 14000000)){
                telemetry.addLine("Gyro broken, please reset robot");
            }
            else{

                double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
                double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                rotX = rotX * 1.1;



                double frontLeftPower = (rotY + rotX + 0) / denominator;
                double rearLeftPower = (rotY - rotX + 0) / denominator;
                double frontRightPower = (rotY - rotX - 0) / denominator;
                double rearRightPower = (rotY + rotX - 0) / denominator;



                feedback = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                feedback = 0;

                // Setting setPoint to desired value
                Direction();

                // Moves collectorMotor

        //        GyroTurn();

                telemetry.addData("Direction", setPoint);

                frontLeftPower -= rx;
                frontRightPower += rx;
                rearLeftPower -= rx;
                rearRightPower += rx;



                if((Math.abs(frontLeftPower) < 0.05) & (Math.abs(rearLeftPower) < 0.05) & (Math.abs(frontRightPower) < 0.05) & (Math.abs(rearRightPower) < 0.05))
                {
                    frontLeftMotor.setPower(0);
                    rearLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    rearRightMotor.setPower(0);
                }
                else
                {
                    frontLeftMotor.setPower(frontLeftPower);
                    rearLeftMotor.setPower(rearLeftPower);
                    frontRightMotor.setPower(frontRightPower);
                    rearRightMotor.setPower(rearRightPower);
                }

                telemetry.addData("Robot current yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

                telemetry.addData("Left Sensor", leftSensor.getDistance(DistanceUnit.CM));
                telemetry.addData("Right Sensor", rightSensor.getDistance(DistanceUnit.CM));

                telemetry.addData("Error", error);
                telemetry.addData("Rotation Speed", rotationSpeed);
                telemetry.update();
            }
        }
    }
    int slidePos = 0;
    double armPos = 0;
    public void AttachmentController()
    {
        double slidePosError, slidePower;
        if(gamepad2.a)
        {
            slidePos = 300;
            armPos = 0.5;
        } else if (gamepad2.b)
        {
            slidePos = 1500;
            armPos = 0.5;
        } else if (gamepad2.x)
        {
            slidePos = 2300;
            armPos = 0.5;
        } else if (gamepad2.y)
        {
            slidePos = 3000;
            armPos = 0.5;
        }

        if(gamepad2.dpad_down)
        {
            slidePos = 0;
            armPos = 0;
        }

        slidePos += -gamepad2.left_stick_y * 15;
        armPos += gamepad2.right_stick_y * 0.05;

        slidePos = Math.min(Math.max(slidePos, 0), 3070);


        if(slideMotor1.getCurrentPosition() < 10 && slidePos < 5)
        {
            slideMotor1.setPower(0);
            slideMotor2.setPower(0);
        }
        else{
            //slideMotor1.setPower(0.3);
            //slideMotor2.setPower(0.3);

            //slideMotor1.setTargetPosition(slidePos);
            //slideMotor2.setTargetPosition(slidePos);

            slidePosError = slidePos - slideMotor1.getCurrentPosition();
            slidePower = slidePosError * 0.9/500;
            slidePower = Math.min(Math.max(slidePower, -0.3), 0.4);

            slideMotor1.setPower(slidePower);
            slideMotor2.setPower(slidePower);
            telemetry.addData("slidePosError", slidePosError);
        }
        telemetry.addData("slideMotor1 Power", slideMotor1.getPower());
        telemetry.addData("slidePosition", slidePos);

        if(gamepad2.left_bumper){
            leftGripper.setPosition(0.5);
        }

        if(gamepad2.right_bumper){
            rightGripper.setPosition(0.5);
        }

        if(gamepad2.right_trigger > 0){
            rightGripper.setPosition(0);
        }

        if(gamepad2.left_trigger > 0){
            leftGripper.setPosition(0);
        }

        arm.setPosition(armPos);
    }

    public void ConfigureG()
    {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
    }

    public void GyroTurn(){
        // Math that rotates robot
        // Offsets error so that robot will turn the right way
        if(setPoint > 180){
            setPoint = setPoint - 360;
        }
        else if(setPoint < -180){
            setPoint = setPoint + 360;
        }
        error = setPoint - feedback;
        if(error > 180){
            error = error - 360;
        }
        else if(error < -180){
            error = error + 360;
        }

        // Rotation sensitivity
        rotationSpeed = 0.04 * error;
        // Keeps turning speed in a range
        if(rotationSpeed > 0.4){
            rotationSpeed = 0.4;
        }

        else if(rotationSpeed < -0.4){
            rotationSpeed = -0.4;
        }
    }

    public void Direction()
    {
        if(gamepad1.dpad_left){
            setPoint = 90;
        }
        else if(gamepad1.dpad_up){
            setPoint = 0;
        }
        else if(gamepad1.dpad_right){
            setPoint = -90;
        }
        else if(gamepad1.dpad_down){
            setPoint = 180;
        }
        else{
            setPoint += rx;
        }
    }
}
