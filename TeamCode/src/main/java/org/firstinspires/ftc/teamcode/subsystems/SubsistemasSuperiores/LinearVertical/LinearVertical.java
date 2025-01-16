package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class LinearVertical {


    public final DcMotorEx motorR;
    public final DcMotorEx motorL;
    public final DigitalChannel led;
    LinearVerticalStates LinearVerticalstates = LinearVerticalStates.Initial;
    // controla descer
    public boolean needToChangeTarget = false;
    public double timeToChangeTarget = 0;
    public int wantedTarget = 0;


    public int position;
    public double power;

    public int targetPosition = 0;
    public static double p = 0.008, i = 0, d = 0.008,f = 0.000;
    PIDController controller = new PIDController(p, i, d);

    /* POSIÇÕES PRESETS */
    private int lowBasketPos = 3500, drivingPos = 200, intakingPos = 10;


    public LinearVertical (HardwareMap hardwareMap) {


        this.motorL =  hardwareMap.get(DcMotorEx.class, "linearL");
        this.motorR =  hardwareMap.get(DcMotorEx.class, "linearR");
        this.led =  hardwareMap.get(DigitalChannel.class, "1ledR");//atualizar no drive hub
        this.power = motorR.getPower();


        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.FORWARD);
        this.reset();
        this.position = motorR.getCurrentPosition();
        this.targetPosition = this.position;
    }

    public void reset() {
        this.motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double PIDF(int target) {

        controller.setPID(p, i,d);
        int linearpos = motorL.getCurrentPosition();
        double pid = controller.calculate(linearpos, target);
        double ff = Math.cos(Math.toRadians(target)) * f;

        return pid;
    }
    public void upSetPoint() {
        this.targetPosition += 80;
    }
    public void downSetPoint() {
        this.targetPosition -= 80;
    }
    public void changeTarget(double runTime, double delay, int target) {
        timeToChangeTarget = runTime + delay;
        needToChangeTarget = true;
        wantedTarget = target;
    }

    public Action ElevadorGoTo(int target) {

        return new Action() {
            int margin;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    started = true;
                }
                targetPosition = target;
                if (target > 2500) {
                    margin = 300;
                }
                else {
                    margin  = 200;
                }
                if (targetPosition < 100 ) {
                    margin  = 30;
                }


                double positionError = 0;
                if(targetPosition > 0 ) {
                    motorR.setPower(PIDF(target));
                    motorL.setPower(PIDF(target));
                    //positionError = PIDF(target);
                }
                else {
                    positionError = PIDF(target);
                    motorL.setPower(PIDF(target));
                    motorR.setPower(PIDF(target));
                }
                if (time.milliseconds() > 800 && Math.abs(motorL.getVelocity()) < 15 && targetPosition < 100 ) {
                    reset();
                    return false;
                }
                if(targetPosition < 700 && Math.abs(positionError) < margin){
                    motorL.setPower(0);
                    motorR.setPower(0);
                }
                return Math.abs(positionError) > margin;
            }

        };

    }

}
