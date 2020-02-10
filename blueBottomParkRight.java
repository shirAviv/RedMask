package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "blueBottomParkRight", group = "Linear Opmode")
public class blueBottomParkRight extends LinearOpMode {

    DcMotor R0, R2, L1, L3, CE1, CE2, SCM;
    private ElapsedTime timer = new ElapsedTime();
    float power = (float) 0.7;

    public void setup() {

        R0 = hardwareMap.get(DcMotor.class, "R0");
        R2 = hardwareMap.get(DcMotor.class, "R2");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L3 = hardwareMap.get(DcMotor.class, "L3");
        CE1 = hardwareMap.get(DcMotor.class, "CE1");//cube eater 1 (right)
        CE2 = hardwareMap.get(DcMotor.class, "CE2");//cube eater 2 (left)
        SCM = hardwareMap.get(DcMotor.class, "SCM"); //seazer motor (misparaim)

        R0.setDirection(DcMotor.Direction.FORWARD);
        R2.setDirection(DcMotor.Direction.FORWARD);
        L1.setDirection(DcMotor.Direction.FORWARD);
        L3.setDirection(DcMotor.Direction.FORWARD);
        CE1.setDirection(DcMotor.Direction.FORWARD);
        CE2.setDirection(DcMotor.Direction.FORWARD);
        SCM.setDirection(DcMotor.Direction.FORWARD);


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


    public void drive(int dir) {

        int target; // the original ticks of the motor ( see datasheet )
        int target_position; //the final position ( ticks )
        int error = 665; //the original ticks  divide by 3 ( 1993/3 = 665 )
        int error2; // the target position - original one circle 1328 ( 1993 -665 )
        double power = 0.2; //power
        double acceleration = 0.01; // by how much the motor accelerate!
        //=================================================================
        target = 1993;//by the datasheet ( see goBilda web Motor 84 RPM )


        //set the direction of the motor
        // 1 --> FWD
        // not 1 --> BWD
        if (dir == 1) {
            L3.setDirection(DcMotor.Direction.FORWARD);
        } else {
            L3.setDirection(DcMotor.Direction.REVERSE);
        }

        //the mode of the motor
        //After using enncoder - do stop and reset
        L3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //mesure the encoder
        L3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //when power is 0 - do a breakes!
        L3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //supply power from the battery
        L3.setPower(0.3);

        //calc the error
        target_position = (L3.getCurrentPosition() + target) - error;
        error2 = target_position - 1328;
        target_position = target_position - error2;

        //run to position
        while (opModeIsActive()&&L3.getCurrentPosition() < target_position) {

            //acceleration
            power = power + acceleration;
            if (power >= 1) {
                //dont over 1!
                // if the power is 1 --> keep it 1
                power = 1;
            }

            //keep update the power
            L3.setPower(power);
        }

        //stop!
        L3.setPower(0);
    }


    public void drive(int dir , int cm) {

        int target; // the original ticks of the motor ( see datasheet )
        int target_position; //the final position ( ticks )
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
                L3.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.REVERSE);
                R0.setDirection(DcMotor.Direction.FORWARD);
                break;
            }
            case 2: {
                L3.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.FORWARD);
                R0.setDirection(DcMotor.Direction.REVERSE);
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
                R0.setDirection(DcMotor.Direction.REVERSE);
                R2.setDirection(DcMotor.Direction.REVERSE);
                L1.setDirection(DcMotor.Direction.REVERSE);
                L3.setDirection(DcMotor.Direction.REVERSE);
                break;
            }
            case 6:{
                R0.setDirection(DcMotor.Direction.FORWARD);
                R2.setDirection(DcMotor.Direction.FORWARD);
                L1.setDirection(DcMotor.Direction.FORWARD);
                L3.setDirection(DcMotor.Direction.FORWARD);
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
        target_position = (L3.getCurrentPosition() + target) - error;
        // be safe with abs value
        //  error2 = Math.abs(target_position - 1328);
        // target_position = Math.abs(target_position - error2);



        //run to position
        while (opModeIsActive()&&(L3.getCurrentPosition() < target_position) ) {


            //acceleration
            power = power + acceleration;
            if (power >= 1) {
                //dont over 1!
                // if the power is 1 --> keep it 1
                power = 1;
            }

            //keep update the power
            R2.setPower(power);
            L3.setPower(power);
            L1.setPower(power);
            R0.setPower(power);

            telemetry.addData("R2 ticks = " , R2.getCurrentPosition());
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


    public void stopCubeEaters(){
        CE1.setPower(0);
        CE2.setPower(0);
    }


    //main
    @Override
    public void runOpMode() throws InterruptedException {

        //init mode
        setup();
        waitForStart();
        //go forward
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
        drive(1,30);
        //drive left
        drive(4, 34);
        //drive forward
        drive(1, 32);
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
        drive(3, 22);
        drive(2, 30);
        //end this should pick a cube from the left side of the Arena. :)



        while (opModeIsActive()) {
        }

    }

}
