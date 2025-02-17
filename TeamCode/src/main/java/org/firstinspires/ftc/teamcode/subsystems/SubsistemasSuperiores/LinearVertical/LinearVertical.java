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
import org.firstinspires.ftc.teamcode.common.PIDTargetChecker;

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
    public static int portaLinearVerticalDireita, portaLinearVerticalEsquerdo, margem = 30, margemAut = 100 , sense = 40;
    public static double tempoParaEstabilizacao = 0.2;
    public PIDTargetChecker pidTargetChecker = new PIDTargetChecker(margem, tempoParaEstabilizacao);
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public int position;
    public double power;
    public static int targetPosition = 0;
    public static double p = 0.001, i = 0, d = 0.000,f = 0, ll = 0, kll = 0;
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
            kp = p * 2;
        }

        if(motorR.getCurrent(CurrentUnit.AMPS) > 2.5 && targetPosition > 2000) {
            targetPosition -= 20;
        }
        if (motorR.getCurrent(CurrentUnit.AMPS) > 3.8) {
            targetPosition = (targetPosition + motorR.getCurrentPosition()) / 2; // Aproxima suavemente do valor atual
        }
        if(this.motorR.getCurrentPosition() < 70 && targetPosition < 10 && targetPosition > -180) {
            motorR.setPower(0);
            motorL.setPower(0);
            return controller.getPositionError();
        }

        controller.setPID(kp, i,d);
        double pid = controller.calculate(linearpos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition)) * f;

        motorL.setPower(pid+ff);
        motorR.setPower(pid+ff);

        return pid + ff;
    }
    public void upSetPoint() {
        changeTarget(targetPosition + sense);
    }
    public void downSetPoint() {
        changeTarget(targetPosition - sense);
    }
    public void changeTarget(int target) {

        targetPosition = target;
        tempoIndoAteOsetPoint.reset();
    }

    public Action ElevadorGoTo(int target) {

        return new Action() {

            ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    targetPosition = target;
                    time.reset();
                    started = true;
                }

                PIDF();

                boolean condicaoDeParada;

                if(target > 0) {
                    condicaoDeParada = motorR.getCurrentPosition() >= targetPosition - margemAut  &&  motorR.getCurrentPosition() <= targetPosition + margemAut ;
                }
                else {
                    condicaoDeParada = motorR.getCurrentPosition() >= targetPosition - margemAut  &&  motorR.getCurrentPosition() <= targetPosition + margemAut ;
                }


                    if(condicaoDeParada) {
                        motorL.setPower(Math.cos(Math.toRadians(target)) * f);
                        motorR.setPower(Math.cos(Math.toRadians(target)) * f);
                        return false;
                    }

                    return true;
            }

        };

    }

    public boolean chegouNoTarget() {
        return pidTargetChecker.hasReachedTarget(targetPosition, motorR.getCurrentPosition());
    }



    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("======================================");
            telemetry.addLine("TELEMETRIA DO BRAÇO DO LINEAR VERTICAL");
            telemetry.addLine("======================================");
            telemetry.addData("VERTICAL-Posição Motor Left: ",motorL.getCurrentPosition());
            telemetry.addData("VERTICAL-Posição Motor Right: ",motorR.getCurrentPosition());
            telemetry.addData("VERTICAL-alvo: ",targetPosition);
            telemetry.addData("VERTICAL-Corrente",motorR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("VERTICAL- chegou na posição alvo e estabilizou", chegouNoTarget());
            telemetry.addData("VERTICAL- getPower", motorR.getPower());
            //telemetry.addData("",);

        }
    }


}
