package istic.gla.groupb.nivimoju.drone.latlong;

import org.apache.log4j.Logger;
import org.junit.Assert;
import org.junit.Test;


/**
 * Test the localCoordinate
 */
public class LocalCoordinateTest {
    Logger logger = Logger.getLogger(LocalCoordinateTest.class);
    private final static double MAX_DEVIATION = 0.001;

    @Test
    public void testConstructors(){
        LocalCoordinate a = new LocalCoordinate();
        logger.info(a.toString());
        Assert.assertEquals(0, a.getX(), MAX_DEVIATION);
        Assert.assertEquals(0, a.getY(), MAX_DEVIATION);
        Assert.assertEquals(0, a.getZ(), MAX_DEVIATION);

        LocalCoordinate b = new LocalCoordinate(1, 2);
        logger.info(b.toString());
        Assert.assertEquals(1, b.getX(), MAX_DEVIATION);
        Assert.assertEquals(2, b.getY(), MAX_DEVIATION);
        Assert.assertEquals(0, b.getZ(), MAX_DEVIATION);

        LocalCoordinate c = new LocalCoordinate(1, 2, 3);
        logger.info(c.toString());
        Assert.assertEquals(1, c.getX(), MAX_DEVIATION);
        Assert.assertEquals(2, c.getY(), MAX_DEVIATION);
        Assert.assertEquals(3, c.getZ(), MAX_DEVIATION);
    }

    @Test
    public void testEquals(){
        LocalCoordinate a = new LocalCoordinate();
        a.setX(1);
        a.setY(2);
        a.setZ(3);
        logger.info(a.toString());
        Assert.assertEquals(1, a.getX(), MAX_DEVIATION);
        Assert.assertEquals(2, a.getY(), MAX_DEVIATION);
        Assert.assertEquals(3, a.getZ(), MAX_DEVIATION);

        LocalCoordinate b = new LocalCoordinate(1, 2, 3);
        logger.info(b.toString());
        Assert.assertEquals(1, b.getX(), MAX_DEVIATION);
        Assert.assertEquals(2, b.getY(), MAX_DEVIATION);
        Assert.assertEquals(3, b.getZ(), MAX_DEVIATION);

        Assert.assertEquals(a, b);

        Assert.assertEquals(a.hashCode(), b.hashCode());
    }
}