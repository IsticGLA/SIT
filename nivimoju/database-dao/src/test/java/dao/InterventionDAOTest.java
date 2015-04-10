package dao;

import entity.Intervention;
import entity.Resource;
import org.junit.*;
import util.State;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by jeremy on 10/04/15.
 */
public class InterventionDAOTest {

    private static Intervention interData;
    private static InterventionDAO interDAO;

    @BeforeClass
    public static void init(){
        interDAO = new InterventionDAO();
        DAOManager.connectTest();
    }

    @AfterClass
    public static void close(){
        interDAO.disconnect();
    }

    @Before
    public void instantiate(){
        List<Resource> ressources = new ArrayList<>();
        ressources.add(new Resource("VSAV", State.planned));
        ressources.add(new Resource("VLCG", State.planned));
        interData = new Intervention("test_insert", 4, 48.11, -1.61);
        interData.setResources(ressources);
    }

    @Test
    public void createTest(){
        Intervention originalIntervention = interDAO.cloneEntity(interData);
        Intervention insertIntervention = interDAO.create(interData);
        originalIntervention.setId(insertIntervention.getId());
        Assert.assertEquals(originalIntervention, insertIntervention);
    }

    @Test
    public void updateTest(){
        Intervention res = interDAO.create(interData);

        res.setName("test_updated");
        res.getResources().add(new Resource("TEST", State.waiting));

        Intervention updateIntervention = interDAO.update(res);
        Assert.assertEquals(res.getResources(), updateIntervention.getResources());
        Assert.assertEquals("test_updated", updateIntervention.getName());
    }

    @Test
    public void deleteTest(){
        Intervention insertIntervention = interDAO.create(interData);
        long id = interDAO.delete(insertIntervention);
        Assert.assertEquals(id, insertIntervention.getId());
        Intervention nullIntervention = interDAO.getById(insertIntervention.getId());
        Assert.assertNull(nullIntervention);
    }

    @Test
    public void getByIdTest(){
        Intervention nullIntervention = interDAO.getById(interData.getId());
        Assert.assertNull(nullIntervention);

        UserDAO uDAO = new UserDAO();

        Intervention insertIntervention = interDAO.create(interData);
        Intervention getByIdIntervention = interDAO.getById(insertIntervention.getId());
        Assert.assertEquals(insertIntervention, getByIdIntervention);
    }

    @Test
    public void getAllTest() throws InterruptedException {

        Intervention st1 = interDAO.create(interData);
        Intervention st2 = interDAO.create(interData);
        Intervention st3 = interDAO.create(interData);
        Intervention st4 = interDAO.create(interData);

        int counter = 0;
        List<Intervention> list = interDAO.getAll();

        for (Intervention st : list){
            System.out.println(st.getId() + "  " + st1.getId());
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
