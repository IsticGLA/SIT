package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.StaticData;
import org.junit.*;
import istic.gla.groupb.nivimoju.util.MarkerType;

import java.util.List;

/**
 * Created by jeremy on 09/04/15.
 */
public class StaticDataDAOTest {

    private static StaticData stData;
    private static StaticData stDataTemp;
    private static StaticDataDAO stDAO;

    @BeforeClass
    public static void init(){
        stDAO = new StaticDataDAO();
        stDAO.connectTest();
        stData = new StaticData(48.5323354, -1.6766565, MarkerType.danger);
        stData = stDAO.create(stData);
    }

    @AfterClass
    public static void close(){
        stDAO.delete(stData);
        stDAO.disconnect();
    }

    @Before
    public void instantiate(){
        stDataTemp = new StaticData(48.666, -1.64000, MarkerType.incident);
    }

    @Test
    public void createTest(){
        // Save the data to insert
        StaticData originalStaticData = stDAO.cloneEntity(stDataTemp);
        // Insert the data in database
        StaticData insertedStaticData = stDAO.create(stDataTemp);
        // Update the id of the original data to match the inserted data
        originalStaticData.setId(insertedStaticData.getId());
        // Assert the equality of datas
        Assert.assertEquals(originalStaticData, insertedStaticData);
        // clean the database
        stDAO.delete(originalStaticData);
    }

    @Test
    public void getByIdTest(){
        StaticData res = stDAO.getById(-1l);
        Assert.assertNull(res);

        res = stDAO.getById(stData.getId());
        Assert.assertEquals(res, stData);
    }

    @Test
    public void getByTest(){
        stData = stDAO.create(stData);
        List<StaticData> res = stDAO.getBy("markerType", MarkerType.danger.toString());
        Assert.assertEquals(stData.getMarkerType(), res.get(0).getMarkerType());
        stDAO.delete(stData);

    }

    @Test
    public void updateTest(){
        long newLat = (long) 47.6334;
        stData.setLatitude(newLat);
        StaticData updateStaticData = stDAO.update(stData);
        Assert.assertEquals(newLat, updateStaticData.getLatitude(), 0.0000);
    }

    @Test
    public void getAllTest() throws InterruptedException {
        // insert three other data
        stDAO.create(stData);
        stDAO.create(stData);
        stDAO.create(stData);

        // check that we have 4 data in database and clean the database
        int counter = 0;
        List<StaticData> list = stDAO.getAll();

        for (StaticData st : list){
            counter++;
            stDAO.delete(st);
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void deleteTest(){
        long id = stDAO.delete(stDataTemp);
        Assert.assertEquals(-1, id);
        stData = stDAO.create(stData);
        id = stDAO.delete(stData);
        Assert.assertEquals(id, stData.getId());
        StaticData nullStaticData = stDAO.getById(stData.getId());
        Assert.assertNull(nullStaticData);
    }
}
