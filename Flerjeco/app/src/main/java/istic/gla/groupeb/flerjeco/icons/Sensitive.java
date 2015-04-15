package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Point;

/**
 * This class allows to draw the representation
 * of a Sensitive point according to the SIT graphic
 */
public class Sensitive {

    private Path triangle;
    private Paint paint;

    public Sensitive(){
        paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setAntiAlias(true);
        changeComponent(Component.HUMAN);
        Point point1 = new Point(0,0);
        Point point2 = new Point(40,0);
        Point point3 = new Point((point2.x+point1.x)/2, (int) ((point2.x-point1.x)*(Math.sqrt(3)/2))+point1.y);
        triangle = new Path();
        triangle.moveTo(point1.x, point1.y);
        triangle.lineTo(point2.x, point2.y);
        triangle.lineTo(point3.x, point3.y);
        triangle.lineTo(point1.x, point1.y);
        triangle.close();
    }

    public enum Component{
        HUMAN, FIRE, RISKS, WATER
    }

    public void changeComponent(Component component){
        switch (component){
            case HUMAN:
                paint.setColor(Color.GREEN);
                break;
            case FIRE:
                paint.setColor(Color.RED);
                break;
            case RISKS:
                paint.setColor(Color.argb(0,255,102,0));
                break;
            case WATER:
                paint.setColor(Color.BLUE);
                break;
            default:
                paint.setColor(Color.BLACK);
        }
    }

    public Path getTriangle() {
        return triangle;
    }

    public Paint getPaint() {
        return paint;
    }

    public void drawSensitive(Canvas mCanvas){
        mCanvas.drawPath(triangle, paint);
    }

}
