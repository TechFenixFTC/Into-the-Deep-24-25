package org.firstinspires.ftc.teamcode.subsystems.common.Horizontal;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.HardwareNames;

@Config
public class LinearHorizontalV4 {
    public DcMotorEx encoder;
    public Servo servoLinearHorizontal;
    public static boolean monitor = false;
    public int targetPositionLinearHorizontal = 0;
    public double erro = 0, p = 0, ll = 0.2, ff = 0;
    public ElapsedTime timeGoingToSetPoint = new ElapsedTime();
    public static double kp = 0.01, kd = 0.000, kff = 0.0000, kll = 0.16, valorMinimoLL = 6, valorMinimoAlertaDeCorrente = 3.25;
    private int codAction = 0;
    public static int portaLinearHorizontalServo;

    public LinearHorizontalV4(HardwareMap hardwareMap) {
        this.servoLinearHorizontal = hardwareMap.get(Servo.class, HardwareNames.horizontalSuperiorServo);
        this.encoder = hardwareMap.get(DcMotorEx.class, "rightBack");

        reset();
        portaLinearHorizontalServo = servoLinearHorizontal.getPortNumber();

    }

    /**************************************************
     *                  Controllers                   *
     **************************************************/
    public double controladorDePosicao() {

        /*
        if(travouTentandoVoltar()) {
            reset();
        }
         */
        // FeedForward
        ff = targetPositionLinearHorizontal * kff;
        //KP
        p = kp;
        //Cria o Controlador PID
        PIDController controller = new PIDController(p, 0, kd);
        //Calcular correção
        double pid = controller.calculate(this.encoder.getCurrentPosition(), targetPositionLinearHorizontal);
        //aqui é um truque matemático pra saber a direção do movimento que meu robo precisa ir com base se o erro é positivo ou negativo
        double direcao = controller.getPositionError() / Math.abs(controller.getPositionError());
        //aqui eu multiplico o valor do lower limit por 1 ou por -1 que ta armazenado na minha variável de direção para acertar a direção da força
        ll = kll * direcao;
        //aqui quero desligar o ll para valores de erro muito pequenos
        if (Math.abs(controller.getPositionError()) <= valorMinimoLL) ll = 0;
        //Mandando a energia pro motor
        //motorLinearHorizontal.setPower(pid + ll);
        return controller.getPositionError();

    }

    /**************************************************
     *                   Actions                      *
     **************************************************/
    /*REFAZER A FUNÇÕES PARA FUNCIONAREM COM SERVOS(SÃO FUNÇÕES BASEADAS NOS MOTORES)*/
    public void reset() {
        setTarget(0);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action extenderLimiteMecanico() {
        codAction = 1;
        return new Action() {
            int margin;
            final ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    setTarget(1200);
                    started = true;
                }

                double positionError = controladorDePosicao();

                boolean condicaoPararBateuLimiteMecanico = time.time() > 0.15 && strangeCurrentBehavior();
                boolean condicaoPararTempoLimite = time.time() > 1;
                boolean condicaoPararOutraAcao = !(codAction == 1);

                if (condicaoPararBateuLimiteMecanico || condicaoPararTempoLimite || condicaoPararOutraAcao) {
                    targetPositionLinearHorizontal = encoder.getCurrentPosition();
                    return false;
                }
                return Math.abs(positionError) > margin;
            }

        };
    }

    public Action retrairLimiteMecanico() {
        codAction = 2;
        return new Action() {
            int margin;
            final ElapsedTime time = new ElapsedTime();
            boolean started = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    setTarget(-1200);
                    started = true;
                }

                double positionError = controladorDePosicao();

                boolean condicaoPararBateuLimiteMecanico = time.time() > 0.15 && strangeCurrentBehavior();
                boolean condicaoPararTempoLimite = time.time() > 1;
                boolean condicaoPararOutraAcao = !(codAction == 2);

                if (condicaoPararBateuLimiteMecanico || condicaoPararTempoLimite || condicaoPararOutraAcao) {
                    reset();
                    return false;
                }
                return Math.abs(positionError) > margin;
            }

        };
    }

    /**************************************************
     *              Controllers Tools                 *
     **************************************************/
    public void setTarget(int target) {
        timeGoingToSetPoint.reset();
        targetPositionLinearHorizontal = target;
    }

    public boolean strangeCurrentBehavior()/*NÃO FUNCIONA MUDAR PARA LOGICA DE SERVOS*/ {
        //return motorLinearHorizontal.getCurrent(CurrentUnit.AMPS) > valorMinimoAlertaDeCorrente;
        return false;
    }

    /**************************************************
     *                   Monitoring                   *
     **************************************************/

    public void monitor(Telemetry telemetry,String hozizontal) {
        if (monitor) {
            telemetry.addLine("*********************************");
            telemetry.addData("TELEMETRIA DO LINEAR HORIZONTAL ",hozizontal);
            telemetry.addLine("*********************************");
            telemetry.addData("-Posição do Servo: ",servoLinearHorizontal.getPosition());
            telemetry.addData("-Posição alvo: ",targetPositionLinearHorizontal);
            telemetry.addData("-Encoder: ",encoder.getCurrentPosition());
        }
    }
}