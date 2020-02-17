package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "blueBottomParkLeft", group = "Linear Opmode")
public class blueBottomParkLeft extends LinearOpMode {
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    Servo CM;
    private ElapsedTime timer = new ElapsedTime();
    float power = (float) 0.7;
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    final double SCALE_FACTOR = 255;


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


        telemetry.addData("Stauts", "success!");
        telemetry.update();
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
            CM.setPosition(0.55);
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
        drive(3, 25);
        drive(2,distance);
        catch_release_cube(false);
    }
    //main
    @Override
    public void runOpMode() throws InterruptedException {


        //init mode
        setup();
        waitForStart();
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);
        //catch_release_cube(true);
        sleep(2000);
        drive(1, 36);
        drive(4, 72);


        sensorColor.enableLed(true);
        //found skystone
        if (sensorColor.green() <= 2000 && sensorColor.red() <= 2000) {
            catch_release_cube(true);
            sensorColor.enableLed(false);
            //fix according to real distance
            move_cube_to_building_zone(150);
            drive(1,150);
            drive(1,60);
            catch_release_cube(true);
            //fix according to real distance
            move_cube_to_building_zone(210);
            //park
            drive(1,40);
        } else {
            drive(1, 20);
            sensorColor.enableLed(true);
            if (sensorColor.green() <= 2000 && sensorColor.red() <= 2000) {
                catch_release_cube(true);
                sensorColor.enableLed(false);
                move_cube_to_building_zone(170);
                drive(1,170);
                drive(1,60);
                catch_release_cube(true);
                //fix according to real distance
                move_cube_to_building_zone(230);
                //park
                drive(1,40);
            } else {
                drive(1, 20);
                catch_release_cube(true);
                move_cube_to_building_zone(190);
                drive(1,190);
                drive(1,60);
                catch_release_cube(true);
                //fix according to real distance
                move_cube_to_building_zone(250);
                //park
                drive(1,40);
            }
        }


//        drive(1,13);
            //catch_release_cube(false);


    /*    //go forward
        drive(1,13);
        //go left
        drive(4, 95);
        //go forward to cube
        drive(1, 7);
        eatCube();
        //go forward while eating cube
        drive(1,6);
        sleep(100);
        stopCubeEaters();
        //go right
        drive(3,40);
        //turn 180 degrees.
        int dist=distanceToTurn(180);
        drive(5,dist);
        //drive forward
        drive(1, 113);
        pukeCube();
        //drive forward while releasing
        drive(1,10);
        //drive backward
        drive(2,6);
        sleep(100);
        stopCubeEaters();
        //end first stone

        //drive back
        drive(2, 96);
        //turn 180 degrees
        dist=distanceToTurn(180);
        drive(6,dist);
        //drive forward
        drive(1,25);
        //drive left
        drive(4, 34);
        //drive forward
        drive(1, 35);
        eatCube();
        //drive forward while eating cube
        drive(1,4);
        sleep(100);
        stopCubeEaters();
        //drive right
        drive(3,40);
        //turn 180 degrees
        distanceToTurn(180);
        drive(5,dist);
        //drive forward
        drive(1, 140);
        pukeCube();
        //drive forward while releasing cube
        drive(1,10);
        sleep(100);
        //drive backward
        drive(2,5);
        sleep(100);
        stopCubeEaters();
        drive(3, 15);
        drive(2, 25);
        drive(4, 35 );
        //end this should pick a cube from the left side of the Arena. :)
*/


            while (opModeIsActive()) {
            }

    }

}

