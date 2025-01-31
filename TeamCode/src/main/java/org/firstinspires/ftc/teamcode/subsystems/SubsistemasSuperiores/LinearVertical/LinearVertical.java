package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.LinearVertical;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;

import java.util.HashMap;

@Config
public class LinearVertical {


    public final DcMotorEx motorR;
    public final DcMotorEx motorL;
    LinearVerticalStates linearVerticalstate = LinearVerticalStates.Initial;
    // controla descer
    public boolean needToChangeTarget = false;
    public double timeToChangeTarget = 0;
    public int wantedTarget = 0;
    public static boolean monitor= false;
    HashMap<LinearVerticalStates, Integer> mapLinearVertical = new HashMap<>();
    public static int portaLinearVerticalDireita, portaLinearVerticalEsquerdo;
    public int position;
    public double power;
    public static int targetPosition = 0;
    public static double p = 0.004, i = 0, d = 0.000,f = 0;
    PIDController controller = new PIDController(p, i, d);

    /* POSIÇÕES PRESETS */
    private int lowBasketPos = 3500, drivingPos = 200, intakingPos = 10;


    public LinearVertical (HardwareMap hardwareMap) {


        this.motorL =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalL);
        this.motorR =  hardwareMap.get(DcMotorEx.class, HardwareNames.verticalR);
        this.power = motorR.getPower();
        portaLinearVerticalDireita = motorR.getPortNumber();
        portaLinearVerticalEsquerdo = motorL.getPortNumber();


        this.motorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motorR.setDirection(DcMotorSimple.Direction.REVERSE);
        this.motorL.setDirection(DcMotorSimple.Direction.REVERSE);
        this.reset();
        this.position = motorR.getCurrentPosition();
        this.targetPosition = this.position;
        mapLinearVertical.put(LinearVerticalStates.Outtake,1000);
    }

    public void reset() {
        this.motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double PIDF() {

        double kp = p;

        int linearpos = motorR.getCurrentPosition();
        if(linearpos < this.targetPosition) {
            kp = p * 8;
        }
        if(linearpos > this.targetPosition + 200){
            kp = p / 4;
        }
        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, this.targetPosition);
        double ff = Math.cos(Math.toRadians(this.targetPosition)) * f;
        motorL.setPower(pid+ff);
        motorR.setPower(pid+ff);

        return pid + ff;
    }
    public void upSetPoint() {
        this.targetPosition += 10;
    }
    public void downSetPoint() {
        this.targetPosition -= 10;
    }
    public void changeTarget(double runTime, double delay, int target) {
        timeToChangeTarget = runTime + delay;
        needToChangeTarget = true;
        wantedTarget = target;
    }

    public Action ElevadorGoTo(int target) {
        this.targetPosition = target;
        return new Action() {
            int margin = 200;
            ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    started = true;
                }

                    PIDF();
                    boolean condicaoDeParada = motorR.getCurrentPosition() >= targetPosition - margin  &&  motorR.getCurrentPosition() <= targetPosition + margin ;


                    if(condicaoDeParada) {
                        return false;
                    }

                    return true;
            }

        };

    }



    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("======================================");
            telemetry.addLine("TELEMETRIA DO BRAÇO DO LINEAR VERTICAL");
            telemetry.addLine("======================================");
            telemetry.addData("-Posição Motor Left: ",motorL.getCurrentPosition());
            telemetry.addData("-Posição Motor Right: ",motorR.getCurrentPosition());
            telemetry.addData("-alvo: ",targetPosition);
            telemetry.addData("-Corrente: ",motorR.getCurrent(CurrentUnit.AMPS));
            //telemetry.addData("",);

        }
    }


}
