package org.firstinspires.ftc.teamcode.subsystems.controls;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SubsistemasInferiores.Sugar.IntakeSuccao;
import org.firstinspires.ftc.teamcode.subsystems.configSample.BlueSample;
import org.firstinspires.ftc.teamcode.subsystems.configSample.RedSample;
import org.firstinspires.ftc.teamcode.subsystems.configSample.YellowSample;

import java.util.ArrayList;
import java.util.List;

@Config
public class MatchColor {


    double[] red60mm = {RedSample.r60mmR, RedSample.r60mmG, RedSample.r60mmB};
    double[] red150mm = {RedSample.r150mmR, RedSample.r150mmG, RedSample.r150mmB};

    double[] blue60mm = {BlueSample.b60mmR, BlueSample.b60mmG, BlueSample.b60mmB};
    double[] blue150mm = {BlueSample.b150mmR, BlueSample.b150mmG, BlueSample.b150mmB};

    double[] yellow60mm = {YellowSample.y60mmR, YellowSample.y60mmG, YellowSample.y60mmB};
    double[] yellow150mm = {YellowSample.y150mmR, YellowSample.y150mmG,YellowSample.y150mmB};


    int currentRed;
    int currentGreen;
    int currentBlue;
    int currentAlpha;
    private List<Integer> allRed = new ArrayList<>();
    private List<Integer> allBlues = new ArrayList<>();
    private List<Integer> allGreen = new ArrayList<>();
    private List<Integer> allAlpha = new ArrayList<>();

    public int media;
    boolean haveSample = false;
    public int c;

    double treshold = 0.64;
    public int getAlpha() {
        return currentAlpha;
    }

    public void addBlue()  {allBlues.add(currentBlue);}
    public void addRed()   {allRed.add(currentRed);}
    public void addGreen() {allGreen.add(currentGreen);}
    public void addAlpha() {allAlpha.add(currentAlpha);}

    public int getMedia(ElapsedTime cronos){
        double time = cronos.seconds();
        if(time < 0.1){
            addAlpha();
            addBlue();
            addGreen();
            addRed();
        }
        for (int i = 0; i < allAlpha.size(); i++) {
             c = allAlpha.get(i);
             
        }
        cronos.reset();

        return media;
    }









    public boolean isRed(double currentRed , double currentGreen,  double currentBlue, double currentAlpha, IntakeSuccao intakeSuccao){

        if(currentRed > RedSample.r60mmR && currentRed < 5000 &&  currentBlue < 1200 && currentAlpha < 2700){
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }
    public boolean isBlue(double currentRed , double currentGreen,  double currentBlue, IntakeSuccao intakeSuccao){

        if(currentBlue > BlueSample.b60mmB &&  currentRed < 700){
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }
    public boolean isYellow(double currentRed , double currentGreen,  double currentBlue, double currentAlpha, IntakeSuccao intakeSuccao){

        if(currentGreen > 4000 && currentAlpha > 2500){
            haveSample = true;
            return true;
        }
        haveSample = false;
        return false;
    }


    /*public String colorSample(double currentRed , double currentGreen,  double currentBlue, IntakeSuccao intakeSuccao){

        /*if(intakeSuccao.colorSensorSugar.getDistance() >= 0.6 && intakeSuccao.colorSensorSugar.getDistance()  <=  0.87) {
            media = (blue60mm[2] / currentBlue);
            if (media > treshold) {
                haveSample = true;
                return "BLUE";
            }

            media = (1 - (yellow60mm[0] / currentRed) +  (1 - (yellow60mm[1] / currentGreen)) + (1 - (yellow60mm[2] / currentBlue))) / 3;
            if (media > treshold) {
                haveSample = true;
                return "YELLOW";
            }

            media = (red60mm[0] / currentRed);
            if (media < treshold) {
                haveSample = true;
                return "RED";
            }
        }
        if(intakeSuccao.colorSensorSugar.getDistance() > 0.87 && intakeSuccao.colorSensorSugar.getDistance() <= 1.54){
            media = ((red150mm[0] / currentRed) + (red150mm[1] / currentGreen) + (red150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "RED";
            }

            media = ((yellow150mm[0] / currentRed) + (yellow150mm[1] / currentGreen) + (yellow150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "YELLOW";
            }

            media = ((blue150mm[0] / currentRed) + (blue150mm[1] / currentGreen) + (blue150mm[2] / currentBlue)) / 3;
            if (media > treshold) {
                haveSample = true;
                return "BLUE";
            }
        }
        haveSample = false;
        return "EMPTY";
    }*/

    public boolean verifyPositionSampleToTransfer(){

        return haveSample;
    }
}