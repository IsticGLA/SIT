package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Point;

import util.ResourceRole;

/**
 * This class allows to draw the representation
 * of a Sensitive point according to the SIT graphic
 */
public class Sensitive implements IIcon {

    private Path triangle;
    private Paint paint;

    public Sensitive(){
        paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setAntiAlias(true);
        changeComponent(ResourceRole.otherwise);
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

    public Sensitive(ResourceRole component){
        paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setAntiAlias(true);
        changeComponent(component);
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

    @Override
    public void drawIcon(Canvas mCanvas) {
        mCanvas.drawPath(triangle, paint);
    }

    public void changeComponent(ResourceRole component){
        switch (component){
            case people:
                paint.setColor(Color.GREEN);
                break;
            case fire:
                paint.setColor(Color.RED);
                break;
            case risks:
                paint.setColor(Color.argb(0,255,102,0));
                break;
            case water:
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

}
