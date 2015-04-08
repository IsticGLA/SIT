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

    public Vehicle(String name){
        this.name = name;
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(10);
        paint.setTextSize(35);
        changeFunction(Function.Default);
        changeState(State.Programmed);
        rect = new Rect(10,100,300,200);
        rect2 = new Rect(rect.centerX()-10, rect.top-50, rect.centerX()+10, rect.top);
    }

    public enum State {Programmed, Validated}

    public enum Function {Water, Fire, People, Risks, Commands, Default}

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
                paint.setColor(Color.argb(0,255,102,0));
                break;
            case Commands:
                paint.setColor(Color.argb(0,153,0,102));
                break;
            default:
                paint.setColor(Color.BLACK);
                break;
        }
    }

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

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Paint getPaint() {
        return paint;
    }

    public Rect getRect() {
        return rect;
    }

    public Rect getRect2() {
        return rect2;
    }
}