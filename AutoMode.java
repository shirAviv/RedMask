package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.graphics.Color.*;


@Autonomous(name = "program 1", group = "Linear Opmode")
public class AutoMode   extends LinearOpMode {





    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor3;
    private DcMotor motor2;
    private ColorSensor sensorColor1;
    private ColorSensor sensorColor2;




    public void setupRobot() {


        telemetry.addData("Status", "wow222");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        motor0 = hardwareMap.get(DcMotor.class, "upperright");
        motor1 = hardwareMap.get(DcMotor.class, "upperleft");
        motor3 = hardwareMap.get(DcMotor.class, "lowerright");
        motor2 = hardwareMap.get(DcMotor.class, "lowerleft");

//        sensorColor1 = hardwareMap.get(ColorSensor.class, "sensor_color1");
//        sensorColor2 = hardwareMap.get(ColorSensor.class, "sensor_color2");

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        /*
         Most robots need the motor on one side to be reversed to drive forward
         Reverse the motor that runs backwards when connected directly to the battery
        */


        waitForStart();
        runtime.reset();

//        telemetry.addData("Status", "Success!!");
//        telemetry.update();


    }


    public void setPower_diagonal_left(double power) {
        motor1.setPower(power);
        motor3.setPower(power);
    }

    public void setPower_diagonal_right(double power) {
        motor0.setPower(power);
        motor2.setPower(power);
    }


    public void setPower(double power) {
        motor1.setPower(power);
        motor0.setPower(power);
        motor2.setPower(power);
        motor3.setPower(power);
    }



    public void redOrBlueStop(double power) {
        int color1 = sensorColor1.argb();
        int color2 = sensorColor2.argb();

        if (color1 == RED || color2 == BLUE) {
            setPower(0);

            motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

    }


    public void runOpMode() {




        setupRobot();


        // Wait for the game to start (driver presses PLAY)


        // run until the end of the match (driver presses STOP)
        waitForStart();

        motor0.setTargetPosition(10); //1993
        motor1.setTargetPosition(-10);
        motor2.setTargetPosition(-10);
        motor3.setTargetPosition(10);


//        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
//        motor3.setDirection(DcMotorSimple.Direction.FORWARD);
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);


        setPower(0.3);

        motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(motor0.isBusy() || motor1.isBusy() || motor2.isBusy() || motor3.isBusy())  {
            telemetry.addData("target pos %.2f",motor2.getTargetPosition());
            telemetry.update();

        }
        setPower(0);


        // Show the elapsed game time and wheel power.
//            telemetry.addData("target pos %.2f",upperleft2.getTargetPosition());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.update();

    }
}



