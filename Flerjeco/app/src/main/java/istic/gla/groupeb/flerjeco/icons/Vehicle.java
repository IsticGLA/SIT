package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Rect;

import util.ResourceRole;
import util.State;

/**
 * This class allows to draw the representation
 * of a Vehicle according to the SIT graphic
 */
public class Vehicle extends Canvas {

    private Paint paint;
    private Rect rect;
    private Rect rect2;
    private String name;
    private State state;
    private ResourceRole role;
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
        this.setState(State.planned);
        this.setRole(ResourceRole.otherwise);
        rect = new Rect(10, 40, 160, 110);
        rect2 = new Rect(rect.centerX()-10, rect.top-30, rect.centerX()+10, rect.top);
    }

    public Vehicle(String name, ResourceRole role){
        this.name = name;
        this.setState(State.planned);
        this.setRole(role);
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(5);
        paint.setTextSize(15);
        rect = new Rect(10, 40, 160, 110);
        rect2 = new Rect(rect.centerX()-10, rect.top-30, rect.centerX()+10, rect.top);
    }

    public Vehicle(String name, ResourceRole role, State state){
        this.name = name;
        this.setRole(role);
        this.setState(state);
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(5);
        paint.setTextSize(15);
        rect = new Rect(10, 40, 160, 110);
        rect2 = new Rect(rect.centerX()-10, rect.top-30, rect.centerX()+10, rect.top);
    }

    /**
     * Method that changes the function of the vehicle
     * @param function the new function of the vehicle
     */
    public void changeFunction(ResourceRole function){
        paint.setStyle(Paint.Style.STROKE);
        switch (function){
            case water:
                paint.setColor(Color.BLUE);
                break;
            case fire:
                paint.setColor(Color.RED);
                break;
            case people:
                paint.setColor(Color.GREEN);
                break;
            case risks:
                paint.setColor(Color.argb(255,255,102,0));
                break;
            case commands:
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
            case planned:
                paint.setPathEffect(new DashPathEffect(new float[]{20, 10}, 0));
                break;
            case active:
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

    public State getState() {
        return state;
    }

    public void setState(State state) {
        this.state = state;
        changeState(state);
    }

    public ResourceRole getRole() {
        return role;
    }

    public void setRole(ResourceRole role) {
        this.role = role;
        changeFunction(role);
    }
}