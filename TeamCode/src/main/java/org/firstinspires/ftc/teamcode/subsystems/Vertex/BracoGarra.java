package org.firstinspires.ftc.teamcode.subsystems.Vertex;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class BracoGarra {
    public Servo servoBracoDaGarra;
    public DcMotorEx motorBracoGarra;
    public int targetPosition = 0;
    public static double kp = 0.00035, ki = 0, kd = 700000, kff = -0.006, kll = 0.055, kmola = 30;
    public static double kpA = 0.05, kiA = 0, kdA = 700000, kffA = -0.006, kllA = 0.055, kmolaA = 30;
    public double ff = 0, pid = 0, ll = 0;
    public double motion = 0;
    public double targetAngle = 0;
    public double angle = 0, angleCalc = 0;
    public double correcao = 0;
    private final double ticks_in_degree = 8192 / 360;
    public boolean overshoot = false;
    public  boolean
            needToGoToIntake = false,
            needToGoToStored = false,
            needToGoToBasketOutake = false,
            needToGoToChamberOutake = false,
            needToGoToIntermadiate = false,
            autonomo = false;

    public static  double
            maxAcc       = 2700,
            maxVelocity  = 4400,
            distance     = 0,
            tempo        = 0;
    public double when   = 0;


    public BracoGarra(HardwareMap hardwareMap) {
        this.servoBracoDaGarra = hardwareMap.get(Servo.class, "porta1");

        servoBracoDaGarra.getController().pwmDisable();
        motorBracoGarra = (DcMotorEx) hardwareMap.get(DcMotor.class, "bracin");
        motorBracoGarra.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBracoGarra.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        reset();
    }

    public void reset() {
        motorBracoGarra.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBracoGarra.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }




    public int getPosition() {
        return -motorBracoGarra.getCurrentPosition();
    }
    public double calculateFF() {
        double ff = Math.cos(Math.toRadians(targetPosition / ticks_in_degree))  * kff;
        angleCalc = Math.cos(Math.toRadians(targetPosition / ticks_in_degree));
        // caso de resistir a mola quando está em baixo
        if (this.targetAngle > -40) {
            ff = ff * kmola;
        }
        return  ff;
    }
    public double calculateLL() {
        double ll =  Math.cos(Math.toRadians(targetPosition / ticks_in_degree)) * kll;
        // caso de resistir a mola quando está em baixo
        if (this.targetAngle > -40) {
            ll = ll * kmola;
        }
        return Math.abs(ll);
    }
    public double controladorDePosicao() {

        // FeedForward

        double ff = targetPosition  * kff;
        if (!(targetPosition > 2000 && targetPosition < 3000)) {
            ff = 0;
        }

        //KP
        double p = kp;
        if (targetPosition > 4000 && getPosition() < targetPosition && Math.abs(motorBracoGarra.getVelocity()) > 10000) {
            p = p/3.1;
            if(Math.abs(motorBracoGarra.getVelocity()) > 10000){
                p = p / 5.2;
            }
        }
        if ( (targetPosition < 4000 && getPosition() > 4000) || targetPosition > 4500 && getPosition() > 5500) {
           // p = p * 4;
        }



        //Cria o Controlador PID
        PIDController controller = new PIDController(kp, 0, kd);
        controller.setPID(kp, 0, kd);

        //Calcular correção
        double pid = controller.calculate(this.getPosition(), targetPosition);
        //calcular o FeedForward




        // LL

        double ll = (Math.abs(controller.getPositionError()) / controller.getPositionError() * kll);


        this.correcao = pid + ff + ll;
        this.motorBracoGarra.setPower(this.correcao);

        return controller.getPositionError();

    }
    public double controladorDePosicaoMotion(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {



        //KP
        double p = kp;




        //Cria o Controlador PID
        PIDController controller = new PIDController(p, 0, kd);
        controller.setPID(kp, 0, kd);

        //Calcular correção
        double pid = 0;
        if(tempo > 0) {
            pid = controller.calculate(this.getPosition(),targetPosition );//motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime
        }
        else {
            //pid = controller.calculate(this.getPosition(), targetPosition);
            pid = controller.calculate(this.getPosition(), targetPosition);
        }
        motion = motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime);
        //calcular o FeedForward

        // FeedForward
        this.targetAngle = targetPosition / ticks_in_degree - 235;

        this.angle = getPosition() / ticks_in_degree - 235;



        double ff = calculateFF();

        if(controller.getPositionError() > 100  ) {
            ff = 0;

        }
        // caso do ff atrapalhar a ir pra posição de 90 graus
        if ((targetAngle > -140 && targetAngle < -70) ) {

            if(controller.getPositionError() > 100  ) {
                ff = 0;

            }
            else {
             //   ff = ff * - 1;
            }

        }
        if(controller.getPositionError() > 400 ) {
            ff = 0;
        }



        // LL

        double ll = (Math.abs(controller.getPositionError()) / controller.getPositionError()) * calculateLL();
        if( (controller.getPositionError() > 2200 || controller.getPositionError() < 100)  &&  targetAngle < -40) {
            ll = 0;
        }
        if( (controller.getPositionError() > 2200 || controller.getPositionError() < 50)  &&  targetAngle > -40) {
            ll = 0;
        }
        if ((targetPosition > 1000 && targetPosition < 3000 ) && (this.getPosition() > 2000 && this.getPosition() < 4000 ) ) {
            p = p * 4;
        }

        this.correcao = pid + ff + ll;

        this.ff = ff;
        this.pid = pid;
        this. ll = ll;


        this.motorBracoGarra.setPower(this.correcao);



        return controller.getPositionError();



    }
    public double controladorDePosicaoAngulo(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {



        //KP
        double p = kpA;

        //Cria o Controlador PID
        PIDController controller = new PIDController(p, 0, kdA);
        controller.setPID(p, 0, kdA);

        //Calcular correção
        this.targetAngle = targetPosition / ticks_in_degree - 235;
        this.angle = getPosition() / ticks_in_degree - 235;
        double error = targetAngle - angle;

        //pid = controller.calculate(this.getPosition(),targetPosition );//motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime
        pid = controller.calculate(this.angle, targetAngle);

        motion = motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime);
        //calcular o FeedForward

        // FeedForward


        double ff = calculateFF();

        if(controller.getPositionError() > 100  ) {
            ff = 0;

        }
        // LL

        this.correcao = pid + ff;

        this.ff = ff;
        this.pid = pid;
        this. ll = ll;


        this.motorBracoGarra.setPower(this.correcao);



        return controller.getPositionError();



    }
    public Action goToIntakePositon(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;

            needToGoToIntake = true;
            return new InstantAction(() -> {});
        }
        return new Action() {
            int margin;
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!started) {
                    time.reset();
                    targetPosition = -6000;
                     if(autonomo) targetPosition = targetPosition  - 240;
                     if(overshoot) targetPosition = targetPosition - 500;
                     // PROFILE
                     BracoGarra.distance = targetPosition;

                    if(getPosition() > 2000 && !autonomo) {
                       // targetPosition = 6300;
                    }
                    started = true;
                }
                margin  = 500;
                // PROFILE
                BracoGarra.tempo = time.time();


                double positionError = controladorDePosicaoMotion(2000, 5000, targetPosition, time.time());
                //
                condition = Math.abs(positionError) > margin;
                if (!condition || time.time() > 3) {
                    motorBracoGarra.setPower(0.1);
                    // PROFILE
                    //tempo = 0;
                    //distance = 0;
                    return false;
                }
                return condition;
            }

        };
    }



    /*public Action goToStored(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;
            needToGoToStored = true;
            return new InstantAction(() -> {});
        }
        return new Action() {
            int margin;
            boolean started = false;
            boolean condition = true;
            ElapsedTime time = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = -1200;
                  //  if (!autonomo) targetPosition = -10000;
                    started = true;

                    // PROFILE
                    distance = targetPosition - getPosition();
                }


                margin = 200;


                // PROFILE
                tempo = time.time();

                double positionError = controladorDePosicaoAngulo(2000, 5000, 0, 0);

                condition = (time.time() > 0.4 && Math.abs(motorBracoGarra.getVelocity()) < 15);
                if (condition) {
                    if(!autonomo) reset();
                    targetPosition = 0;

                    // PROFILE
                    tempo = 0;
                    distance = 0;
                }
                return !condition;
            }
        };
    }*/
    //goToRetin
    public Action goToStored(double runtime, double delay) {
        if(delay > 0){
            when = runtime + delay;
            needToGoToIntermadiate = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            boolean condition = true;
            ElapsedTime time = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = -1200;
                    started = true;
                    // PROFILE
                    distance = targetPosition - getPosition();
                }


                margin = 100;

                // PROFILE
                tempo = time.time();
                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava posicao de angulo
                condition =  Math.abs(positionError) > margin;

                if (!condition)  {
                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(calculateFF());
                }

                return condition;
            }
        };
    }

    public Action goToIntermediatePosition(double runtime, double delay) {
        if(delay > 0){
            when = runtime + delay;
            needToGoToIntermadiate = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            boolean condition = true;
            ElapsedTime time = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = -3600;
                    started = true;
                    // PROFILE
                    distance = targetPosition - getPosition();
                }


                margin = 100;

                // PROFILE
                tempo = time.time();
                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava posicao de angulo
                condition =  Math.abs(positionError) > margin;

                if (!condition)  {
                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(calculateFF());
                }

                return condition;
            }
        };
    }
    public Action goToRetin(double runtime, double delay) {
        if(delay > 0){
            when = runtime + delay;
            needToGoToIntermadiate = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            boolean condition = true;
            ElapsedTime time = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = 2600;
                    started = true;
                    // PROFILE
                    distance = targetPosition - getPosition();
                }


                margin = 100;

                // PROFILE
                tempo = time.time();
                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava posicao de angulo
                condition =  Math.abs(positionError) > margin;

                if (!condition)  {
                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(calculateFF());
                }

                return condition;
            }
        };
    }
    public Action goFromIntermediateToOutake(double runtime, double delay) {
        if(delay > 0){
            when = runtime + delay;
            needToGoToIntermadiate = true;
            return new InstantAction(() -> {});
        }
        return  new SequentialAction(
                goTo150(0, 0),
                goToStored(0, 0),
                goToBasketOutake(0, 0)
        );
    }
    public Action goToBasketOutake(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;
            needToGoToBasketOutake = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = -2550;
                    if (autonomo) targetPosition += 370;
                    // PROFILE
                    distance = targetPosition - getPosition();
                    started = true;
                }


                margin = 200;

                // PROFILE
                tempo = time.time();

                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava  estava posicaoAngulador
                condition = Math.abs(positionError) > margin;

                if (!condition || time.time() > 0.9) {
                   // motorBracoGarra.setPower(-0.15);
                   // targetPosition = 1500;

                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(-0.12);
                    return false;
                }
                return condition;
            }
        };
    }
    public Action goTo150(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;
            needToGoToBasketOutake = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = 300;
                    // PROFILE
                    distance = targetPosition - getPosition();
                    started = true;
                }


                margin = 200;

                // PROFILE
                tempo = time.time();

                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                condition = Math.abs(positionError) > margin;

                if (!condition || time.time() > 0.9) {
                    // motorBracoGarra.setPower(-0.15);
                    // targetPosition = 1500;

                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(-0.12);
                    return false;
                }
                return condition;
            }
        };
    }
    public Action goToPark(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;
            needToGoToBasketOutake = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = 2350;
                    // PROFILE
                    distance = targetPosition - getPosition();
                    started = true;
                }


                margin = 200;

                // PROFILE
                tempo = time.time();

                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava posicao de angulo
                condition = Math.abs(positionError) > margin;

                if (!condition || time.time() > 0.9) {
                    // motorBracoGarra.setPower(-0.15);
                    // targetPosition = 1500;

                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(-0.12);
                    return false;
                }
                return condition;
            }
        };
    }
    public Action goTotouchBar() {
        return new Action() {
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    started = true;
                }

                condition = true;
                motorBracoGarra.setPower(0.7);
                if (time.time() > 4.9) {
                    // motorBracoGarra.setPower(-0.15);
                    // targetPosition = 1500;
                    motorBracoGarra.setPower(0);
                    return false;
                }
                return condition;
            }
        };

    }

    public Action goToChamberOutake(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + 0.7;
            needToGoToStored = true;
            return new InstantAction(() -> {});
        }

        return goToIntermediatePosition(0, 0);
    }


    public Action handleBracoTeleop(double runtime) {
        if ( needToGoToIntake && runtime >= when) {
            needToGoToIntake = false;
            return goToIntakePositon(runtime, 0);
        }

        if ( needToGoToBasketOutake && runtime >= when) {
            needToGoToBasketOutake = false;
            return goToBasketOutake(runtime, 0);
        }

        if ( needToGoToStored && runtime >= when) {
            needToGoToStored = false;
            return goToStored(runtime, 0);
        }

        if ( needToGoToIntermadiate && runtime >= when) {
            needToGoToIntermadiate  = false;
            return goToIntermediatePosition(runtime, 0);
        }
        //controladorDePosicaoMotion(2000, 5000, 0, 0);
        controladorDePosicao();
        return new InstantAction(() -> {});
    }

    public void upBraco(int quanto) {

        targetPosition = targetPosition + quanto ;


    }
    public void downBraco(int quanto) {
        targetPosition = targetPosition - quanto;
}

    public static double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        // Calcula o tempo necessário para atingir a velocidade máxima
        double accelerationDt = maxVelocity / maxAcceleration;

        // Verifica se é possível acelerar até a velocidade máxima na distância dada
        double halfwayDistance = distance / 2.0;
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        if (accelerationDistance > halfwayDistance) {
            accelerationDt = Math.sqrt(halfwayDistance / (0.5 * maxAcceleration));
        }

        accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationDt, 2);

        // Recalcula a velocidade máxima com base no tempo disponível para acelerar/desacelerar
        maxVelocity = maxAcceleration * accelerationDt;

        // Deceleração ocorre na mesma taxa que a aceleração
        double decelerationDt = accelerationDt;

        // Calcula o tempo em que a velocidade máxima é mantida
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseDt = cruiseDistance / maxVelocity;
        double decelerationTime = accelerationDt + cruiseDt;

        // Calcula o tempo total do perfil de movimento
        double entireDt = accelerationDt + cruiseDt + decelerationDt;
        if (elapsedTime > entireDt) {
            return distance; // Se o tempo decorrido excede o perfil, retorna a distância total
        }

        // Se estamos na fase de aceleração
        if (elapsedTime < accelerationDt) {
            // Equação do movimento com aceleração constante
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }
        // Se estamos na fase de cruzeiro
        else if (elapsedTime < decelerationTime) {
            double cruiseCurrentDt = elapsedTime - accelerationDt;
            return accelerationDistance + maxVelocity * cruiseCurrentDt;
        }
        // Se estamos na fase de desaceleração
        else {
            double decelerationElapsed = elapsedTime - decelerationTime;
            return accelerationDistance + cruiseDistance
                    + maxVelocity * decelerationElapsed
                    - 0.5 * maxAcceleration * Math.pow(decelerationElapsed, 2);
        }
    }
    public Action goToBasketOutakeSample3(double runtime, double delay) {
        if ( delay > 0) {
            when = runtime + delay;
            needToGoToBasketOutake = true;
            return new InstantAction(() -> {});
        }
        return  new Action() {
            int margin;
            boolean started = false;
            ElapsedTime time = new ElapsedTime();
            boolean condition;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!started) {
                    time.reset();
                    targetPosition = 2050;
                    if (autonomo) targetPosition += 450;
                    // PROFILE
                    distance = targetPosition - getPosition();
                    started = true;
                }


                margin = 200;

                // PROFILE
                tempo = time.time();

                double positionError = controladorDePosicaoMotion(2000, 5000, 0, 0);
                // antes estava  estava posicaoAngulador
                condition = Math.abs(positionError) > margin;

                if (!condition || time.time() > 0.9) {
                    // motorBracoGarra.setPower(-0.15);
                    // targetPosition = 1500;

                    // PROFILE
                    tempo = 0;
                    distance = 0;
                    motorBracoGarra.setPower(-0.12);
                    return false;
                }
                return condition;
            }
        };
    }

}