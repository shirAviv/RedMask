package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "blueBottomParkLeft", group = "Linear Opmode")
public class blueBottomParkLeft extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo CM;
    BNO055IMU imu;
    Orientation angle;
    private ElapsedTime timer = new ElapsedTime();
    float power = (float) 0.7;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;

    public void setup_gyro() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            telemetry.addData("calibrating imu", "...");
            telemetry.update();
            sleep(50);
            idle();
        }

    }

    public void reset_gyro()
    {
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("calibrating imu", "...");
            telemetry.update();
            sleep(50);
            idle();
        }
    }

    public void setup() {

        R0 = hardwareMap.get(DcMotor.class, "R0");
        R2 = hardwareMap.get(DcMotor.class, "R2");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L3 = hardwareMap.get(DcMotor.class, "L3");
        CE1 = hardwareMap.get(DcMotor.class, "CE1");//cube eater 1 (right)
        CE2 = hardwareMap.get(DcMotor.class, "CE2");//cube eater 2 (left)
        SCM = hardwareMap.get(DcMotor.class, "SCM"); //seazer motor (misparaim)
        CM = hardwareMap.get(Servo.class, "CM");
        sensorColor = hardwareMap.get(ColorSensor.class, "color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color");

        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.FORWARD);
        CE1.setDirection(DcMotor.Direction.FORWARD);
        CE2.setDirection(DcMotor.Direction.FORWARD);
        SCM.setDirection(DcMotor.Direction.FORWARD);
        CM.setDirection(Servo.Direction.FORWARD);
        sensorColor.enableLed(true);
        setup_gyro();


        telemetry.addData("Stauts", "success!");
        telemetry.update();
    }

    public void turn_by_gyro(int dir, double degrees) {
        //0 == left
        //1 == right

        reset_gyro();

        double error = 13;
        if (dir == 1) {
            R0.setDirection(DcMotor.Direction.FORWARD);
            R2.setDirection(DcMotor.Direction.FORWARD);
            L1.setDirection(DcMotor.Direction.FORWARD);
            L3.setDirection(DcMotor.Direction.FORWARD);


        } else {
            //left
            R0.setDirection(DcMotor.Direction.REVERSE);
            R2.setDirection(DcMotor.Direction.REVERSE);
            L1.setDirection(DcMotor.Direction.REVERSE);
            L3.setDirection(DcMotor.Direction.REVERSE);
        }


        power =(float) 0.95; //do not change!!!!
        L3.setPower(power);
        L1.setPower(power);
        R2.setPower(power);
        R0.setPower(power);
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("the first angle = ", angle.firstAngle);
        telemetry.update();
        double original_angle=angle.firstAngle;
        //calc the error
        if(dir==0) {
            degrees = original_angle+degrees - error;
            while (angle.firstAngle < degrees) {

                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("the first angle = ", angle.firstAngle);
                telemetry.addData("angle = ", angle.firstAngle);
                telemetry.addData("degrees = ",degrees);
                telemetry.update();

            }
            power = 0;
            L3.setPower(power);
            L1.setPower(power);
            R2.setPower(power);
            R0.setPower(power);
        }else{
            degrees = original_angle-(degrees - error);
            while (angle.firstAngle > degrees) {

                angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                telemetry.addData("the first angle = ", angle.firstAngle);
                telemetry.addLine();
                telemetry.addData("angle = ", angle.firstAngle);
                telemetry.addLine();
                telemetry.addData("degrees = ",degrees);
                telemetry.update();

            }
            power = 0;
            L3.setPower(power);
            L1.setPower(power);
            R2.setPower(power);
            R0.setPower(power);
        }
    }


    public void setMotionEnginesMotorPower(float power) {
        R0.setPower(power);
        R2.setPower(power);
        L1.setPower(power);
        L3.setPower(power);
    }

    public void goFwd() {
        R0.setDirection(DcMotor.Direction.REVERSE);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.REVERSE);
        L3.setDirection(DcMotor.Direction.FORWARD);
        setMotionEnginesMotorPower(power);
    }


    public void drive(int dir, int cm) {

        int target; // the original ticks of the motor ( see datasheet )
        int target_position_L1; //the final position ( ticks )
        int target_position_L3; //the final position ( ticks )
        int target_position_R0; //the final position ( ticks )
        int target_position_R2; //the final position ( ticks )

        int error = 0; //the original ticks  divide by 3 ( 1993/3 = 665 )
        int error2; // the target position - original one circle 1328 ( 1993 -665 )
        double power = 0.7; //power
        double acceleration = 0.01; // by how much the motor accelerate!
        target = cm * 45;//by the datasheet ( see goBilda web Motor 84 RPM )
        //45 ticks is 1 cm

        //set the direction of the motor
        // 1 --> FWD
        // 2 --> BWD
        //3 --> RIGHT
        //4 --> LEFT
        //5--> ROTATE LEFT
        //6--> ROTATE RIGHT
        switch (dir) {
            case 1: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 2: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 3: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 4: {
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
                break;

            }

            case 5: {
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 6: {
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
        }

        //the mode of the motor
        //After using enncoder - do stop and reset
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        L1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //mesure the encoder
        L3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        R0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //when power is 0 - do a breakes!
        L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        R0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //supply power from the battery
        L3.setPower(power);
        L1.setPower(power);
        R2.setPower(power);
        R0.setPower(power);

        //calc the error - by ticks
        target_position_L3 = (L3.getCurrentPosition() + target) - error;
        target_position_L1 = (L3.getCurrentPosition() + target) - error;
        target_position_R0 = (L3.getCurrentPosition() + target) - error;
        target_position_R2 = (L3.getCurrentPosition() + target) - error;

        // be safe with abs value
        //  error2 = Math.abs(target_position - 1328);
        // target_position = Math.abs(target_position - error2);


        //run to position
        while (opModeIsActive() && (L3.getCurrentPosition() < target_position_L3) ) {


            //acceleration

            /*
            power = power + acceleration;
            if (power >= 0.9) {
//                dont over 1!
//                 if the power is 1 --> keep it 1
                power = 0.9;
            }
            */

            //keep update the power
            R2.setPower(power);
            L3.setPower(power);
            L1.setPower(power);
            R0.setPower(power);

            telemetry.addData("R2 ticks = ", R2.getCurrentPosition());
            telemetry.update();
        }

        //stop!
        L3.setPower(0);
        L1.setPower(0);
        R2.setPower(0);
        R0.setPower(0);

    }

    private int distanceToTurn(int degrees) {
        int wheel_diamater = 10;
        int len_betweeen_wheels = 48;
        int amountToTurn = (int) len_betweeen_wheels * degrees / (2 * wheel_diamater);
        //4 motors- divide by 4
        amountToTurn = amountToTurn / 4;
        return amountToTurn;
    }

    private void eatCube() {


        //=================================================================

        //bring cube inside
        CE1.setDirection((DcMotorSimple.Direction.FORWARD));
        CE2.setDirection((DcMotorSimple.Direction.REVERSE));


        //supply power from the battery
        CE1.setPower(power);
        CE2.setPower(power);

    }

    private void pukeCube() {


        //==============================s===================================

        //bring cube outside
        CE1.setDirection((DcMotorSimple.Direction.REVERSE));
        CE2.setDirection((DcMotorSimple.Direction.FORWARD));

        CE1.setPower(power);
        CE2.setPower(power);


    }


    public void stopCubeEaters() {
        CE1.setPower(0);
        CE2.setPower(0);
    }

    public void catch_release_cube(boolean lower) {

        telemetry.addData("CM position", CM.getPosition());
        telemetry.update();
        if (lower) {
            CM.setDirection(Servo.Direction.REVERSE);
            CM.setPosition(0.60);
            telemetry.addData("CM position after low", CM.getPosition());
            telemetry.update();
        } else {
            CM.setDirection(Servo.Direction.REVERSE);
            CM.setPosition(0.05);
            telemetry.addData("CM position after up", CM.getPosition());
            telemetry.update();
        }
        sleep(100);

//        CM.setDirection(Servo.Direction.FORWARD);
    }


    public void move_cube_to_building_zone(int distance) {
//        drive(3, 25);
//        int dist = distanceToTurn(110); //110 degrees = 90
//        drive(6, dist);
        turn_by_gyro(1,90);
        telemetry.addData("moving to building zone","...");
        telemetry.update();
        sleep(2000);
        drive(4, distance);
        catch_release_cube(false);
    }
    //main
    @Override
    public void runOpMode() throws InterruptedException {

        //init mode
        setup();
        waitForStart();
        //drive to the cube
        drive(1,25);
        drive(4, 60);
//        sensorColor.enableLed(true);
        telemetry.addData("sensor dist ",sensorDistance.getDistance(DistanceUnit.CM));
        telemetry.update();
        telemetry.addData("green1",sensorColor.green());
        telemetry.addData("red1",sensorColor.red());
        telemetry.update();
        sleep(1000);
        if (sensorColor.green() <= 300 && sensorColor.red() <= 200) {
            telemetry.addData("found ",sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
//            while (sensorDistance.getDistance(DistanceUnit.CM)>2) {
//                drive(4,1);
//                sleep(200);
//            }
            drive(4,1);
            sleep(200);
            drive(4,1);
            sleep(200);
            drive(4,1);
            sleep(200);
            drive(4,1);
            sleep(200);

            //sky stone
            sleep(250);
            catch_release_cube(true);
            sleep(250);

            sensorColor.enableLed(false);
            drive(3, 30);
            move_cube_to_building_zone(120);
            drive(3, 168);
            turn_by_gyro(1, 90);

            drive(4, 30);
            while (sensorDistance.getDistance(DistanceUnit.CM)>3) {
                drive(4,1);
                sleep(200);
            }
            drive(4,1);

            sleep(250);
            catch_release_cube(true);
            sleep(250);

            drive(3, 20);
            turn_by_gyro(0,15);

            move_cube_to_building_zone(180);
            drive(3, 55);
            drive(2,35);
        } else {
            drive(1, 20);
            sensorColor.enableLed(true);
            telemetry.addData("sensor dist ",sensorDistance.getDistance(DistanceUnit.CM));
            telemetry.update();
//            sleep(1000);
//            while (sensorDistance.getDistance(DistanceUnit.CM)>5) {
//                drive(4,1);
//                sleep(1000);
//            }
            telemetry.addData("green2",sensorColor.green());
            telemetry.addData("red2",sensorColor.red());
            telemetry.update();
            sleep(1000);
            if (sensorColor.green() <= 300 && sensorColor.red() <= 200) {
                telemetry.addData("found #2 ",sensorDistance.getDistance(DistanceUnit.CM));
                telemetry.update();
//                while (sensorDistance.getDistance(DistanceUnit.CM)>2) {
//                    drive(4,1);
//                    sleep(200);
//                }
                drive(4,1);
                sleep(200);
                drive(4,1);
                sleep(200);
                drive(4,1);
                sleep(200);

                sleep(250);
                catch_release_cube(true);
                sleep(250);
                sensorColor.enableLed(false);
                drive(3, 30);
                move_cube_to_building_zone(130);
                drive(3, 160);
                //turn first so that we dont hit the wall
                turn_by_gyro(0, 95);
                drive(1, 8);

                drive(4, 30);
                while (sensorDistance.getDistance(DistanceUnit.CM)>3) {
                    drive(4,1);
                    sleep(200);
                }
                drive(4,1);
                sleep(250);
                catch_release_cube(true);
                sleep(250);
                drive(3, 20);
                turn_by_gyro(0,15);

                move_cube_to_building_zone(180);
                drive(3, 55);
                drive(2,30);
            } else {
                drive(1, 18);
//                sensorColor.enableLed(true);
//                if (sensorColor.green() <= 2000 && sensorColor.red() <= 2000) {
                while (sensorDistance.getDistance(DistanceUnit.CM)>2) {
                    drive(4,1);
                    sleep(200);
                }
                sleep(250);
                catch_release_cube(true);
                sleep(250);
//                    sensorColor.enableLed(false);
                drive(3, 15);
                move_cube_to_building_zone(160);
                drive(3, 55);
                drive(2,30);

//                drive(3, 40);
//                catch_release_cube(true);
//                move_cube_to_building_zone(210);
//                drive(3, 50);
            }
        }

        while (opModeIsActive()) {

        }

    }
}

