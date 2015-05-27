package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Area;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.util.MarkerType;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 10/04/15.
 */
public class InterventionDAOTest {
    private static Intervention iData;
    private static Intervention iDataTemp;
    private static InterventionDAO iDAO;

    @BeforeClass
    public static void init() {
        iDAO = new InterventionDAO();
        iDAO.connectTest();
        //iDAO.connect();
        iData = new Intervention("Test", 2, 48.6445, -1.68454);
        iData = iDAO.create(iData);
    }

    @AfterClass
    public static void close() {
        iDAO.delete(iData);
        iDAO.disconnect();
    }

    @Before
    public void instantiate() {
        iDataTemp = new Intervention("Coucou", 3, 48.6445, -1.68454);
    }

    @Test
    public void createTest() {
        // Save the data to insert
        Intervention originalIntervention = iDAO.cloneEntity(iDataTemp);
        // Insert the data in database
        Intervention insertedIntervention = iDAO.create(iDataTemp);
        // Update the id of the original data to match the inserted data
        originalIntervention.setId(insertedIntervention.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalIntervention, insertedIntervention);
        // clean the database
        iDAO.delete(originalIntervention);
    }

    @Test
    public void getByIdTest() {
        Intervention res = iDAO.getById(-1l);
        Assert.assertNull(res);

        res = iDAO.getById(iData.getId());
        Assert.assertEquals(res, iData);

//        Intervention res = iDAO.getById(268l);
//        Assert.assertNotNull(res);
//        List<Area> areas = new ArrayList<>();
//        List<Position> positions = new ArrayList<>();
//        positions.add(new Position(48.115434, -1.638722));
//        positions.add(new Position(48.115710, -1.637843));
//        positions.add(new Position(48.115194, -1.638143));
//        Area a = new Area(3l);
//        a.setPositions(positions);
//        areas.add(a);
//        res.setWatchArea(areas);
//        Assert.assertEquals(268l, iDAO.update(res));
    }

    @Test
    public void getByTest() {
        iData = iDAO.create(iData);
        List<Intervention> res = iDAO.getBy("incidentCode", 2);
        Assert.assertEquals(iData.getIncidentCode(), res.get(0).getIncidentCode());

        res = iDAO.getBy("creationDate", iData.getCreationDate());
        Assert.assertEquals(iData.getCreationDate(), res.get(0).getCreationDate(), 0.00000);

        iDAO.delete(iData);

    }

    @Test
    public void updateTest() {
        long newLat = (long) 47.6334;

        // Update an unexisting data
        iDataTemp.setLatitude(newLat);
        Intervention unexiiData = iDAO.update(iDataTemp);
        Assert.assertNull(unexiiData);

        // Update existing data
        iData.setLatitude(newLat);
        Intervention updateIntervention = iDAO.update(iData);
        Assert.assertEquals(newLat, updateIntervention.getLatitude(), 0.0000);
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        iDAO.create(iData);
        iDAO.create(iData);
        iDAO.create(iData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<Intervention> list = iDAO.getAll();

        for (Intervention st : list) {
            counter++;
            iDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest() {
        long id = iDAO.delete(iDataTemp);
        Assert.assertEquals(-1, id);
        iData = iDAO.create(iData);
        id = iDAO.delete(iData);
        Assert.assertEquals(id, iData.getId());
        Intervention nullIntervention = iDAO.getById(iData.getId());
        Assert.assertNull(nullIntervention);
    }
}
