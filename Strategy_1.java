package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.graphics.Color.*;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


@Autonomous(name = "program 1", group = "Linear Opmode")
public class Strategy_1 extends LinearOpMode {


        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motor0;
    private DcMotor motor1;
    private DcMotor motor3;
    private DcMotor motor2;
    private DcMotor liftMotor;
    private Servo shelfServo;
    private Servo servo2;
    private double curr_power = 0.7;
    private double shelf_counter = 0;
    private double arm_counter = 0;
    private double shelf_increment = 0.09;
    private double arm_increment = 0.09;


    private ColorSensor sensorColor1;
    private ColorSensor sensorColor2;




        public void setupRobot() {


            telemetry.addData("Status", "wow222");
            telemetry.update();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            motor0 = hardwareMap.get(DcMotor.class, "ur");//upper right
            motor1 = hardwareMap.get(DcMotor.class, "ul"); //upper left
            motor3 = hardwareMap.get(DcMotor.class, "lr"); //lower right
            motor2 = hardwareMap.get(DcMotor.class, "ll"); //lower left
            liftMotor = hardwareMap.get(DcMotor.class,"sm");//
            shelfServo = hardwareMap.get(Servo.class, "ssh");//servo shelf
            servo2 = hardwareMap.get(Servo.class, "sa");//servo arm

            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



//            motor0.setDirection(DcMotorSimple.Direction.FORWARD);
//            motor1.setDirection(DcMotorSimple.Direction.FORWARD);
//            motor2.setDirection(DcMotorSimple.Direction.FORWARD);
//            motor3.setDirection(DcMotorSimple.Direction.FORWARD);

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
            motor0.setPower(power);
            motor1.setPower(power);
            motor2.setPower(power);
            motor3.setPower(power);
        }

        public void moveArmServo(boolean forward, double arm_increment) {
            if (forward) {
                arm_counter = arm_counter + arm_increment;
            } else {
                arm_counter = arm_counter - arm_increment;
            }
            servo2.setPosition(arm_counter);
            sleep(100);
        }

        public void moveShelfServo(boolean forward,double shelf_increment) {
            if (forward) {
                shelf_counter = shelf_counter + shelf_increment;
            } else {
                shelf_counter = shelf_counter - shelf_increment;
            }
            shelfServo.setPosition(shelf_counter);
            sleep(100);
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
//            moveShelfUpDown();
            shelfServo.setPosition(0);
            servo2.setPosition(0);
//            moveShelfServo(TRUE,shelf_increment);
//            sleep(1000);
//            moveShelfServo(FALSE,shelf_increment);
            moveArmServo(TRUE, arm_increment);
            sleep(1000);
            moveArmServo(FALSE, arm_increment);



//            motor0.setTargetPosition(10); //1993
//            motor1.setTargetPosition(10);
//            motor2.setTargetPosition(10);
//            motor3.setTargetPosition(10);
//
//            setPower(0.3);
//
//            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
            while(motor0.isBusy() || motor1.isBusy() || motor2.isBusy() || motor3.isBusy())  {
                telemetry.addData("m 0 target pos %.2f",motor0.getCurrentPosition());
                telemetry.addData("m 1 target pos %.2f",motor1.getCurrentPosition());
                telemetry.addData("m 2 target pos %.2f",motor2.getCurrentPosition());
                telemetry.addData("m 3 target pos %.2f",motor3.getCurrentPosition());
                telemetry.update();

            }
            setPower(0);


                                // Show the elapsed game time and wheel power.
//            telemetry.addData("target pos %.2f",upperleft2.getTargetPosition());
//            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
//            telemetry.update();

        }

    public void moveShelfUpDown() {
        telemetry.addData("automose mode","");
        telemetry.update();

        liftMotor.setTargetPosition(5000);
        liftMotor.setPower(curr_power);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("liftMotor start %.2f", liftMotor.getCurrentPosition());
        telemetry.update();
        sleep(100);

        while (liftMotor.isBusy()) {
            telemetry.addData("m s current pos %.2f", liftMotor.getCurrentPosition());
            telemetry.update();
            sleep(100);
        }
        liftMotor.setPower(0);
    }
}



