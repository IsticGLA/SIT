package istic.gla.groupb.nivimoju.drone.latlong;

import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Position;


import org.apache.log4j.Logger;
import org.junit.*;
import org.junit.rules.ExpectedException;


/**
 * Test the converter of latlong <-> localCoord
 */
public class LatLongConverterTest {
    Logger logger = Logger.getLogger(LatLongConverterTest.class);
    private final static double MAX_DEVIATION = 1;

    /**
     * test plusieurs conversion latlong -> local
     * @throws Exception
     */
    @Test
    public void testGetLocal() throws Exception {
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        logger.info(converter.toString());

        Position center = new Position((48.1222+48.1119)/2, (-1.6428+-1.6337)/2);
        LocalCoordinate expectedCenterLocal = new LocalCoordinate(0, 0, 0);

        logger.info("converting center");
        LocalCoordinate centerLocal = converter.getLocal(center);
        logger.info("centerLocal " + centerLocal);
        Assert.assertEquals(expectedCenterLocal.getX(), centerLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedCenterLocal.getY(), centerLocal.getY(), MAX_DEVIATION);

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

    /**
     * teste qu'une tentative de conversion en dehors de la zone renvois une exception
     * @throws Exception
     */
    @Rule
    public ExpectedException exception = ExpectedException.none();
    @Test
    public void testGetLocalOutsideBounds() throws Exception {
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);
        Position outside = new Position(46, -1);
        exception.expect(IllegalArgumentException.class);
        LocalCoordinate converted = converter.getLocal(outside);
    }

    /**
     * teste plusieurs conversions local -> latlong
     * @throws Exception
     */
    @Test
    public void testGetLatLong() throws Exception {
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);

        Position center = new Position((48.1222+48.1119)/2, (-1.6428+-1.6337)/2);
        LocalCoordinate expectedCenterLocal = new LocalCoordinate(0, 0, 0);

        logger.info("converting center");
        LocalCoordinate centerLocal = converter.getLocal(center);
        logger.info("centerLocal " + centerLocal);
        Assert.assertEquals(expectedCenterLocal.getX(), centerLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedCenterLocal.getY(), centerLocal.getY(), MAX_DEVIATION);

        logger.info("converting croisementArche");
        Position croisementArche = new Position(48.11721 , -1.63888);
        LocalCoordinate croisementArcheLocal = converter.getLocal(croisementArche);
        logger.info("croisementArcheLocal : " + croisementArcheLocal);
        LocalCoordinate expectedcroisementArcheLocal = new LocalCoordinate(-46.733, 16.8795, 0);
        logger.info("delta croisement : " + expectedcroisementArcheLocal.distanceInPlan(croisementArcheLocal));
        Assert.assertEquals(expectedcroisementArcheLocal.getX(), croisementArcheLocal.getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcroisementArcheLocal.getY(), croisementArcheLocal.getY(), MAX_DEVIATION);

        logger.info("converting to latlong");
        Position croisementArcheConverted = converter.getLatLong(expectedcroisementArcheLocal);
        logger.info("expected " + croisementArche);
        logger.info("actual " + croisementArcheConverted);
    }

    /**
     * teste la conversion d'un chemin
     */
    @Test
    public void testGetLocalPath(){
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);

        Position croisementArche = new Position(48.11721 , -1.63888);
        LocalCoordinate expectedcroisementArcheLocal = new LocalCoordinate(-46.733, 16.8795, 0);
        Position parking = new Position(48.12211 , -1.64180);
        LocalCoordinate expectedparkingLocal = new LocalCoordinate(-264.1185, 563.0921, 0);
        Position corner1 = new Position(48.11554 , -1.63920);
        LocalCoordinate expectedcorner1Local = new LocalCoordinate(-70.5016, -168.0714, 0);
        Path path = new Path();
        path.addPosition(croisementArche);
        path.addPosition(parking);
        path.addPosition(corner1);
        path.setClosed(true);

        LocalPath localPath = converter.getLocalPath(path);

        Assert.assertEquals("the path should be closed", true, localPath.isClosed());
        Assert.assertEquals("There sould be 3 points", 3, localPath.getPositions().size());
        Assert.assertEquals(expectedcroisementArcheLocal.getX(), localPath.getPositions().get(0).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcroisementArcheLocal.getY(), localPath.getPositions().get(0).getY(), MAX_DEVIATION);

        Assert.assertEquals(expectedparkingLocal.getX(), localPath.getPositions().get(1).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedparkingLocal.getY(), localPath.getPositions().get(1).getY(), MAX_DEVIATION);

        Assert.assertEquals(expectedcorner1Local.getX(), localPath.getPositions().get(2).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcorner1Local.getY(), localPath.getPositions().get(2).getY(), MAX_DEVIATION);
    }

    /**
     * teste la conversion d'un chemin avec une position en dehors du perimetre de travail
     */
    @Test
    public void testGetLocalPathOutside(){
        LatLongConverter converter = new LatLongConverter(48.1222, -1.6428, 48.1119, -1.6337, 720, 1200);

        Position croisementArche = new Position(48.11721 , -1.63888);
        LocalCoordinate expectedcroisementArcheLocal = new LocalCoordinate(-46.733, 16.8795, 0);
        Position parking = new Position(48.12211 , -1.64180);
        LocalCoordinate expectedparkingLocal = new LocalCoordinate(-264.1185, 563.0921, 0);
        Position corner1 = new Position(48.11554 , -1.63920);
        LocalCoordinate expectedcorner1Local = new LocalCoordinate(-70.5016, -168.0714, 0);
        Position oustide = new Position(46 , -1);
        Path path = new Path();
        path.addPosition(croisementArche);
        path.addPosition(parking);
        path.addPosition(corner1);
        path.addPosition(oustide);
        path.setClosed(true);

        LocalPath localPath = converter.getLocalPath(path);

        Assert.assertEquals("the path should be closed", true, localPath.isClosed());
        Assert.assertEquals("There should be 3 points (no outside)", 3, localPath.getPositions().size());
        Assert.assertEquals(expectedcroisementArcheLocal.getX(), localPath.getPositions().get(0).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcroisementArcheLocal.getY(), localPath.getPositions().get(0).getY(), MAX_DEVIATION);

        Assert.assertEquals(expectedparkingLocal.getX(), localPath.getPositions().get(1).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedparkingLocal.getY(), localPath.getPositions().get(1).getY(), MAX_DEVIATION);

        Assert.assertEquals(expectedcorner1Local.getX(), localPath.getPositions().get(2).getX(), MAX_DEVIATION);
        Assert.assertEquals(expectedcorner1Local.getY(), localPath.getPositions().get(2).getY(), MAX_DEVIATION);
    }
}