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
    private static StaticDataDAO stDAO;

    @BeforeClass
    public static void init(){
        stDAO = new StaticDataDAO();
        DAOManager.connectTest();
    }

    @AfterClass
    public static void close(){
        stDAO.disconnect();
    }

    @Before
    public void instantiate(){
        stData = new StaticData(48.666, -1.64000, MarkerType.danger);
    }

    @Test
    public void createTest(){
        StaticData originalStaticData = stDAO.cloneEntity(stData);
        StaticData insertStaticData = stDAO.create(stData);
        originalStaticData.setId(insertStaticData.getId());
        Assert.assertEquals(originalStaticData, insertStaticData);
    }

    @Test
    public void updateTest(){
        StaticData res = stDAO.create(stData);
        res.setLatitude(47.6334);
        StaticData updateStaticData = stDAO.update(res);
        Assert.assertEquals(47.6334, updateStaticData.getLatitude(), 0.0000);
    }

    @Test
    public void deleteTest(){
        StaticData insertStaticData = stDAO.create(stData);
        long id = stDAO.delete(insertStaticData);
        Assert.assertEquals(id, insertStaticData.getId());
        StaticData nullStaticData = stDAO.getById(insertStaticData.getId());
        Assert.assertNull(nullStaticData);
    }

    @Test
    public void getByIdTest(){
        StaticData nullStaticData = stDAO.getById(stData.getId());
        Assert.assertNull(nullStaticData);

        StaticData insertStaticData = stDAO.create(stData);
        StaticData getByIdStaticData = stDAO.getById(insertStaticData.getId());
        Assert.assertEquals(insertStaticData, getByIdStaticData);
    }

    @Test
    public void getAllTest() throws InterruptedException {

        StaticData st1 = stDAO.create(stData);
        StaticData st2 = stDAO.create(stData);
        StaticData st3 = stDAO.create(stData);
        StaticData st4 = stDAO.create(stData);

        int counter = 0;
        List<StaticData> list = stDAO.getAll();

        for (StaticData st : list){
            if ((st.getId() == st1.getId()) ||
                    (st.getId() == st2.getId()) ||
                    (st.getId() == st3.getId()) ||
                    (st.getId() == st4.getId())) {
                counter++;
            }
        }
        Assert.assertEquals(4, counter);
    }
}