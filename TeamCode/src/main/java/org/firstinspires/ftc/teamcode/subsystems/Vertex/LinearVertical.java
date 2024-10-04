package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;

@Config
public class LinearVertical {
    public final DcMotorEx motorR;
    public final DcMotorEx motorL;
    public int position;
    public double power;

    public int targetPosition = 0;
    public static double p = 0.008, i = 0, d = 0.008, f = 0.000;

    /* POSIÇÕES PRESETS */
    private int lowBasketPos = 2400, drivingPos = 200, intakingPos = 10;


    public LinearVertical (HardwareMap hardwareMap) {
        this.motorL =  hardwareMap.get(DcMotorEx.class, "linearL");
        this.motorR =  hardwareMap.get(DcMotorEx.class, "linearR");
        this.power = motorR.getPower();

        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
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
    public double PIDF(Telemetry telemetry) {

        double kp = p;
        // FeedForward
        if (this.motorR.getCurrentPosition() < 1200) {
            f = f/2;
        }
        double ff = targetPosition  * f;
        //KP
        if (this.targetPosition < 200 && (this.motorR.getCurrentPosition() > 1000) ) {
            kp = p/3;
        }
        //Cria o Controlador PID
        PIDController controller = new PIDController(kp, i, d);
        controller.setPID(kp, i, d);


        //Calcular correção
        double pid = controller.calculate(this.motorR.getCurrentPosition(), targetPosition);

        // turn off the motors if the linear is between 600 and 1150 ticks and error is low
        if(Math.abs(controller.getPositionError()) < 200 && (this.motorR.getCurrentPosition() > 600) && (this.motorR.getCurrentPosition() < 1150) ) {
            motorR.setPower(0);
            motorL.setPower(0);
            return controller.getPositionError();
        }

        motorR.setPower(pid + ff);
        motorL.setPower(pid + ff);
        return controller.getPositionError();

    }
    public void upSetPoint() {
        this.targetPosition += 40;
    }
    public void downSetPoint() {
        this.targetPosition -= 40;
    }



}
