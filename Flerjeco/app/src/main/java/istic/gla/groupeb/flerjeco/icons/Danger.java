package istic.gla.groupeb.flerjeco.icons;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.Point;

import istic.gla.groupb.nivimoju.util.ResourceRole;

/**
 * This class allows to draw the representation
 * of a danger according to the SIT graphic
 */
public class Danger implements IIcon {

    private Path triangle;
    private Paint paint;

    public Danger(){
        paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setAntiAlias(true);
        changeComponent(ResourceRole.otherwise);
        Point point1 = new Point(5,35);
        Point point2 = new Point(35,35);
        Point point3 = new Point((point2.x+point1.x)/2, point1.x);
        triangle = new Path();
        triangle.moveTo(point1.x, point1.y);
        triangle.lineTo(point2.x, point2.y);
        triangle.lineTo(point3.x, point3.y);
        triangle.lineTo(point1.x, point1.y);
        triangle.close();
    }

    public Danger(ResourceRole component){
        paint = new Paint();
        paint.setStrokeWidth(2);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setAntiAlias(true);
        changeComponent(component);
        Point point1 = new Point(0,40);
        Point point2 = new Point(40,40);
        Point point3 = new Point((point2.x+point1.x)/2, point1.x);
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
                paint.setColor(Color.rgb(255,102,0));
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
