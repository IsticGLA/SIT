package istic.gla.groupb.nivimoju.drone.latlong;

import entity.Position;
import junit.framework.Assert;
import junit.framework.TestCase;
import org.apache.log4j.Logger;

/**
 * Created by sacapuces on 09/04/15.
 */
public class LatLongConverterTest extends TestCase {
    Logger logger = Logger.getLogger(LatLongConverterTest.class);
    private final static double MAX_DEVIATION = 1;

    public void testGetLocal() throws Exception {
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        Position topLeft = new Position(48.1222, -1.6428);
        Position bottomRight = new Position(48.1119, -1.6337);
        Position topRight = new Position(48.1222, -1.6337);
        Position bottomLeft = new Position(48.1119, -1.6428);
        Position center = new Position((48.1222+48.1119)/2, (-1.6428+-1.6337)/2);

        LocalCoordinate expectedTopLeftLocal = new LocalCoordinate(-(720/2), (1200/2), 0);
        LocalCoordinate expectedBottomRightLocal = new LocalCoordinate((720/2), -(1200/2), 0);
        LocalCoordinate expectedTopRightLocal = new LocalCoordinate((720/2), (1200/2), 0);
        LocalCoordinate expectedBottomLeftLocal = new LocalCoordinate(-(720/2), -(1200/2), 0);
        LocalCoordinate expectedCenterLocal = new LocalCoordinate(0, 0, 0);

        logger.info("converting center");
        LocalCoordinate centerLocal = converter.getLocal(center);
        logger.info("centerLocal " + centerLocal);
        Assert.assertEquals(expectedCenterLocal.getX(), centerLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedCenterLocal.getY(), centerLocal.getY(), MAX_DEVIATION);
/*
        logger.info("converting topLeft");
        LocalCoordinate topLeftLocal = converter.getLocal(topLeft);
        logger.info("topLeftLocal " + topLeftLocal);
        Assert.assertEquals(expectedTopLeftLocal.getX(), topLeftLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedTopLeftLocal.getY(), topLeftLocal.getY(), MAX_DEVIATION);

        logger.info("converting bottomRight");
        LocalCoordinate bottomRightLocal = converter.getLocal(bottomRight);
        logger.info("bottomRightLocal " + bottomRightLocal);
        Assert.assertEquals(expectedBottomRightLocal.getX(), bottomRightLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedBottomRightLocal.getY(), bottomRightLocal.getY(), MAX_DEVIATION);

        logger.info("converting topRight");
        LocalCoordinate topRightLocal = converter.getLocal(topRight);
        logger.info("topRightLocal " + topRightLocal);
        Assert.assertEquals(expectedTopRightLocal.getX(), topRightLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedTopRightLocal.getY(), topRightLocal.getY(), MAX_DEVIATION);

        logger.info("converting bottomLeft");
        LocalCoordinate bottomLeftLocal = converter.getLocal(bottomLeft);
        logger.info("bottomLeftLocal " + bottomLeftLocal);
        Assert.assertEquals(expectedBottomLeftLocal.getX(), bottomLeftLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedBottomLeftLocal.getY(), bottomLeftLocal.getY(), MAX_DEVIATION);*/

        logger.info("converting croisementArche");
        Position croisementArche = new Position(48.11721 , -1.63888);
        LocalCoordinate croisementArcheLocal = converter.getLocal(croisementArche);
        logger.info("croisementArcheLocal : " + croisementArcheLocal);
        LocalCoordinate expectedcroisementArcheLocal = new LocalCoordinate(-46.733, 16.8795, 0);
        logger.info("delta croisement : " + expectedcroisementArcheLocal.distanceInPlan(croisementArcheLocal));
        Assert.assertEquals(expectedcroisementArcheLocal.getX(), croisementArcheLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcroisementArcheLocal.getY(), croisementArcheLocal.getY(), MAX_DEVIATION);

        logger.info("converting parking");
        Position parking = new Position(48.12211 , -1.64180);
        LocalCoordinate parkingLocal = converter.getLocal(parking);
        logger.info("parkingLocal : " + parkingLocal);
        LocalCoordinate expectedparkingLocal = new LocalCoordinate(-264.1185, 563.0921, 0);
        logger.info("delta parking : " + expectedparkingLocal.distanceInPlan(parkingLocal));
        Assert.assertEquals(expectedparkingLocal.getX(), parkingLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedparkingLocal.getY(), parkingLocal.getY(), MAX_DEVIATION);

        logger.info("converting corner BAT D");
        Position corner1 = new Position(48.11554 , -1.63920);
        LocalCoordinate corner1Local = converter.getLocal(corner1);
        logger.info("corner1Local : " + corner1Local);
        LocalCoordinate expectedcorner1Local = new LocalCoordinate(-70.5016, -168.0714, 0);
        logger.info("delta corner1 : " + expectedcorner1Local.distanceInPlan(corner1Local));
        Assert.assertEquals(expectedcorner1Local.getX(), corner1Local.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcorner1Local.getY(), corner1Local.getY(), MAX_DEVIATION);

    }
}