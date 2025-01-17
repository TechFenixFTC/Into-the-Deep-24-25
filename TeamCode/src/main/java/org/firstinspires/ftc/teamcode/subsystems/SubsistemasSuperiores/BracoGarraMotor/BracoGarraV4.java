package org.firstinspires.ftc.teamcode.subsystems.SubsistemasSuperiores.BracoGarraMotor;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareNames;
import org.firstinspires.ftc.teamcode.common.verifys.PIDTargetChecker;

@Config
public class BracoGarraV4 {
    public static boolean monitor;
    public DcMotorEx motorBracoGarra;
    private double targetAngle = 0;
    public static double kp = 0.025, ki = 0, kd = 0.2, kff = 0.05, kll = 0.0,  margemEmGraus = 6 , tempoDeEstabilidade = 0.2, sense = 4, llMax = kll, maxStep = 5;
    public double ff = 0, pid = 0, ll = 0, angle = 0, correcao = 0, error = 0;
    private final double ticks_in_degree = 8192.0 / 360;
    private double actionFinalTarget = 0;
    public ElapsedTime tempoIndoAteOsetPoint = new ElapsedTime();
    public double tempoUltimaacao = 0;
    private final Telemetry telemetry;
    public PIDTargetChecker pidTargetChecker = new PIDTargetChecker(margemEmGraus, tempoDeEstabilidade);
    public static int portaBracoGarraServo, portaBracoGarraMotor;

    BracoGarraStates bracoGarraState = BracoGarraStates.Initial;
    public static  double   maxAcc = 2700, maxVelocity  = 4400, distance = 0, tempo = 0;
    public BracoGarraV4(HardwareMap hardwareMap, Telemetry telemetry) {
        motorBracoGarra = (DcMotorEx) hardwareMap.get(DcMotor.class, HardwareNames.bracoGarraSuperior);
        motorBracoGarra.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBracoGarra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.telemetry = telemetry;
        reset();
        portaBracoGarraMotor = motorBracoGarra.getPortNumber();
    }
    /**************************************************
     *                  Controllers                   *
     **************************************************/
    public double controladorDePosicao(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
       // Creating the PID controller
        PIDController controller = new PIDController(kp, ki, kd);
       // PID equation
        pid = controller.calculate(getAngle(),  targetAngle);
        error = controller.getPositionError();
       // FeedForward
        calculateFF();
       // LowerLimit
        calculateLL();
       // Final correction equation
        this.correcao = pid + this.ff + this.ll;
       // Energizing the motors
        this.motorBracoGarra.setPower(correcao);
       // Returns the error
        return controller.getPositionError();
    }
    /**************************************************
     *                   Actions                      *
     **************************************************/
    public void reset() {
        motorBracoGarra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBracoGarra.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public Action goToAnyTarget(double finalTarget) {
        this.actionFinalTarget = finalTarget;
        return new Action() {
            int margin;
            double currentTarget;

            double maxStep =  25; // Passo máximo para o ajuste gradual
            final ElapsedTime time = new ElapsedTime();
            boolean started = false;


            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!started) {
                    time.reset();
                    currentTarget = getAngle(); // Começa a partir do ângulo atual do braço
                    started = true;
                    kp = 0.015;
                }

               // Reduzir o maxStep quando ficar perto de posições críticas
                if ((finalTarget < 120 && currentTarget <= 160) || (finalTarget > 170 && currentTarget >= 150)) {
                    maxStep = 6;
                    kp = 0.025;
                }

                // Ajuste gradual do target
                if (Math.abs(currentTarget - finalTarget) > maxStep) {
                    if (currentTarget < finalTarget) {
                        currentTarget += maxStep;
                    } else {
                        currentTarget -= maxStep;
                    }
                } else {
                    currentTarget = finalTarget; // Se estiver próximo o suficiente, define o target final
                }

                setTarget(currentTarget); // Atualiza o target do braço para o ângulo intermediário

                // Controlador de posição e condição de parada
                double positionError = controladorDePosicao(0, 0, 0, 0);
                boolean condicaoParar = Math.abs(error) < margin + 9 && currentTarget == finalTarget;

                boolean condicaoPararOutraAcao = actionFinalTarget != finalTarget;
                // Telemetria para debug
                telemetry.addLine("Indo até o target");

                if (condicaoParar || condicaoPararOutraAcao) {
                    telemetry.addLine("Cheguei no target");
                    return false; // Encerra a ação
                }

                return true; // Continua ajustando
            }
        };
    }
    /**************************************************
     *              Controllers Tools                 *
     **************************************************/
    public void setTarget(double targetAngle) {
        tempoIndoAteOsetPoint.reset();
        this.targetAngle = targetAngle;
    }
    public double getTarget() {
        return targetAngle;
    }
    public int getPosition() {return -motorBracoGarra.getCurrentPosition();}
    public void calculateFF() {
        this.ff = Math.cos(Math.toRadians(getPosition() / ticks_in_degree))  * kff;
        if(  error > 20  ) {
            ff = 0;
        }
    }
    public void calculateLL() {

        if( !chegouNoTarget()  && error < 30) {
            ll = (kll * 1.2 * tempoIndoAteOsetPoint.time()) * error / Math.abs(error);
            ll = Math.max(-kll, Math.min(kll, ll));
        }
        else {
            ll = 0;
            if (tempoIndoAteOsetPoint.time() > 0.3) {
                tempoUltimaacao = tempoIndoAteOsetPoint.time();
            }
            tempoIndoAteOsetPoint.reset();
        }

        if(targetAngle >= 120 && targetAngle <= 200) {
            ll = 0;
        }
    }
    public double getAngle() {
        this.angle = getPosition() / ticks_in_degree + 275;
        return  this.angle;
    }
    public boolean chegouNoTarget() {
        return pidTargetChecker.hasReachedTarget(targetAngle, getAngle());
    }
    public void monitor(Telemetry telemetry) {
        if (monitor) {
            telemetry.addLine("==============================");
            telemetry.addLine("  TELEMETRIA DO BRAÇO DA GARRA");
            telemetry.addLine("===============================");
            telemetry.addData("-Angulo do braco: ",this.getAngle());
            telemetry.addData("-alvo: ",targetAngle);
            //telemetry.addData("",);

        }
    }

}
