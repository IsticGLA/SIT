package istic.gla.groupb.nivimoju.dao;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Resource;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;
import istic.gla.groupb.nivimoju.util.State;

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
        DAOManager.connect();
    }

    @AfterClass
    public static void close(){
        interDAO.disconnect();
    }

    @Before
    public void instantiate(){
        List<Resource> ressources = new ArrayList<>();
        ressources.add(new Resource(1l, "VSAV", State.planned));
        ressources.add(new Resource(2l, "VLCG", State.planned));
        interData = new Intervention("test_insert", 4, 48.11, -1.61);
        interData.setResources(ressources);
    }

    /*@Test
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
        res.getResources().add(new Resource(1l, "TEST", State.waiting));

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
            if ((st.getId() == st1.getId()) ||
                    (st.getId() == st2.getId()) ||
                    (st.getId() == st3.getId()) ||
                    (st.getId() == st4.getId())) {
                counter++;
            }
        }
        Assert.assertEquals(4, counter);
    }

    @Test
    public void getWaitingResourcesTest(){
        interData.getResources().add(new Resource(1l, "VSAP", State.waiting));
        interDAO.create(interData);
        List<Intervention> list = interDAO.getWaitingResources();
        boolean ok = true;
        for (Intervention i : list){
            boolean waiting = false;
            for (Resource r : i.getResources()){
                if (r.getState() == State.waiting){
                    waiting = true;
                    break;
                }
            }
            ok = ok && waiting;
        }
        Assert.assertTrue(ok);
    }*/

    @Test
    public void updateResource(){
        InterventionDAO interventionDAO = new InterventionDAO();


        Intervention intervention = interventionDAO.getById(43L);

        intervention.updateDate();

        List<Resource> resourceList = new ArrayList<>();

        resourceList.add(new Resource(1L, "VSAV1", State.active));

        intervention.setResources(resourceList);

        interventionDAO.update(intervention);


        interventionDAO.getNewerLastUpdate();
    }
}