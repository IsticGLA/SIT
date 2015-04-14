package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.DashPathEffect;
import android.graphics.Paint;
import android.graphics.Rect;

import util.ResourceRole;
import util.State;

/**
 * This class allows to draw the representation
 * of a Vehicle according to the SIT graphic
 */
public class Vehicle extends Canvas {

    public static final int RECT_LEFT = 0;
    public static final int RECT_TOP = 20;
    public static final int RECT_RIGHT_COEF = 8;
    public static final int RECT_RIGHT_SUP = 40;
    public static final int RECT_BOTTOM = 50;
    public static final int RECT_STROKE = 2;
    public static final int RECT2_SIZE = 10;
    public static final int TEXT_SIZE = 15;


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
        initDrawingVehicle(ResourceRole.otherwise, State.planned);
    }

    public Vehicle(String name, ResourceRole role){
        this.name = name;
        initDrawingVehicle(role, State.planned);
    }

    public Vehicle(String name, ResourceRole role, State state){
        this.name = name;
        initDrawingVehicle(role, state);
    }

    private void initDrawingVehicle(ResourceRole role, State state){
        paint = new Paint();
        paint.setAntiAlias(true);
        paint.setStrokeWidth(RECT_STROKE);
        paint.setTextSize(TEXT_SIZE);
        rect = new Rect(RECT_LEFT, RECT_TOP, name.length()*RECT_RIGHT_COEF + RECT_RIGHT_SUP, RECT_BOTTOM);
        rect2 = new Rect(rect.centerX()-(RECT2_SIZE/2), rect.top-RECT2_SIZE, rect.centerX()+(RECT2_SIZE/2), rect.top);
        this.setRole(role);
        this.setState(state);
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
                paint.setPathEffect(new DashPathEffect(new float[]{10, 5}, 0));
                break;
            case active:
                paint.setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
                break;
            default:
                paint.setPathEffect(new DashPathEffect(new float[]{10, 5}, 0));
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

    public void drawVehicle(Canvas mCanvas){
        DashPathEffect temp = (DashPathEffect) paint.getPathEffect();
        Paint tempPaint = new Paint();
        tempPaint.setColor(Color.WHITE);
        tempPaint.setStyle(Paint.Style.FILL);
        mCanvas.drawRect(rect, tempPaint);
        //Drawing the first rectangle
        mCanvas.drawRect(rect, paint);
        //Drawing the second little rectangle
        paint.setStyle(Paint.Style.FILL);
        mCanvas.drawRect(rect2, paint);
        //Drawing the name of the vehicle
        paint.setPathEffect(new DashPathEffect(new float[]{0, 0}, 0));
        mCanvas.drawText(name, rect.centerX() - 40, rect.centerY(), paint);
        paint.setStyle(Paint.Style.STROKE);
        //Reapplying the PathEffect
        paint.setPathEffect(temp);
    }
}