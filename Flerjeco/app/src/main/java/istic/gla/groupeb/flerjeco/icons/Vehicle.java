package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;

/**
 * This class allows to draw the representation
 * of a Vehicle according to the SIT graphic
 */
public class Vehicle extends Canvas {

    private Paint paint;
    private Rect rect;
    private Rect rect2;
    private String name;

    /**
     * Default constructor of a vehicle
     * @param name the name of the vehicle
     */
    public Vehicle(String name){
        this.name = name;
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(5);
        paint.setTextSize(15);
        changeFunction(Function.Default);
        changeState(State.Programmed);
        rect = new Rect(10, 40, 160, 110);
        rect2 = new Rect(rect.centerX()-10, rect.top-30, rect.centerX()+10, rect.top);
    }

    public Vehicle(String name, Function function){
        this.name = name;
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(5);
        paint.setTextSize(15);
        changeFunction(function);
        changeState(State.Programmed);
        rect = new Rect(10, 40, 160, 110);
        rect2 = new Rect(rect.centerX()-10, rect.top-30, rect.centerX()+10, rect.top);
    }

    public enum State {Programmed, Validated}

    public enum Function {Water, Fire, People, Risks, Commands, Default}

    /**
     * Method that changes the function of the vehicle
     * @param function the new function of the vehicle
     */
    public void changeFunction(Function function){
        paint.setStyle(Paint.Style.STROKE);
        switch (function){
            case Water:
                paint.setColor(Color.BLUE);
                break;
            case Fire:
                paint.setColor(Color.RED);
                break;
            case People:
                paint.setColor(Color.GREEN);
                break;
            case Risks:
                paint.setColor(Color.argb(255,255,102,0));
                break;
            case Commands:
                paint.setColor(Color.argb(255,153,0,102));
                break;
            default:
                paint.setColor(Color.BLACK);
                break;
        }
    }

    /**
     * Method that changes the state of the vehicle
     * @param state the new state of the vehicle
     */
    public void changeState(State state){
        paint.setStyle(Paint.Style.STROKE);
        switch (state){
            case Programmed:
                paint.setPathEffect(new DashPathEffect(new float[]{20, 10}, 0));
                break;
            case Validated:
                paint.setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
                break;
            default:
                paint.setPathEffect(new DashPathEffect(new float[]{20, 10}, 0));
                break;
        }
    }

    /**
     * Getter for the vehicle's name
     * @return the name of the vehicle
     */
    public String getName() {
        return name;
    }

    /**
     * Setter for the vehicle's name
     * @param name the new name of the vehicle
     */
    public void setName(String name) {
        this.name = name;
    }

    /**
     * Getter for the paint graphics of the vehicle
     * @return the paint graphics
     */
    public Paint getPaint() {
        return paint;
    }

    /**
     * Getter for the first big rectangle
     * @return the first big rectangle
     */
    public Rect getRect() {
        return rect;
    }

    /**
     * Getter for the second little rectangle
     * @return the second little rectangle
     */
    public Rect getRect2() {
        return rect2;
    }
}