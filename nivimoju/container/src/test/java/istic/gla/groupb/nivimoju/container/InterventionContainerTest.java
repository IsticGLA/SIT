package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.dao.InterventionDAO;
import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.State;
import org.apache.log4j.Logger;
import org.junit.AfterClass;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import java.util.Collection;

/**
 * Created by vivien on 26/05/15.
 */
public class InterventionContainerTest {
    Logger logger = Logger.getLogger(InterventionContainerTest.class);
    InterventionContainer container;

    @Before
    public void init(){
        InterventionContainer.destroy();
        // Connect to the test database
        container = InterventionContainer.getInstanceTest();
    }

    @AfterClass
    public static void cleanDatabase(){
        InterventionDAO interDAO = new InterventionDAO();
        interDAO.connectTest();

        InterventionContainer container = InterventionContainer.getInstanceTest();

        Collection<Intervention> interList = container.getInterventions();

        for(Intervention inter : interList) {
            interDAO.delete(inter);
        }
    }

    @Test
    public void testCreateIntervention(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test1", 1, 48.0, -1.6), true);

        Assert.assertEquals("inter_test1", interventionContainer.getName());
        Assert.assertEquals(48.0, interventionContainer.getLatitude(), 0);
        Assert.assertEquals(-1.6, interventionContainer.getLongitude(), 0);
        Assert.assertEquals(1, interventionContainer.getIncidentCode());
    }

    @Test
    public void testUpdateIntervention(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test2", 2, 48.0, -1.6), true);

        interventionContainer.setName("inter_test2_updated");
        interventionContainer = container.updateIntervention(interventionContainer);

        Assert.assertEquals("inter_test2_updated", interventionContainer.getName());
    }

    @Test
    public void testChangeResourceState(){
        Intervention intervention = new Intervention("inter_test3", 3, 48.0, -1.6);
        Resource res = new Resource();
        intervention.getResources().add(0, res);
        Intervention interventionContainer = container.createIntervention(intervention, true);

        interventionContainer = container.changeResourceState(interventionContainer.getId(), 0L, State.arrived.toString());

        Assert.assertEquals(State.arrived, interventionContainer.getResources().get(0).getState());
    }

    @Test
    public void testAddVehicle(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);

        interventionContainer = container.addVehicle(interventionContainer.getId(), "vehicle_test");
        interventionContainer = container.addVehicle(interventionContainer.getId(), "vehicle_test2");
        Resource vehicle = interventionContainer.getResources().get(0);

        Assert.assertEquals("vehicle_test" + vehicle.getIdRes(), vehicle.getLabel());
        Assert.assertEquals(ResourceCategory.vehicule, vehicle.getResourceCategory());
    }

    @Test
    public void testAddResource(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);

        Resource resource = new Resource();
        resource.setResourceCategory(ResourceCategory.dragabledata);
        resource.setLabel("res_test");
        interventionContainer = container.addResource(interventionContainer.getId(), resource);

        Resource resource2 = new Resource();
        resource2.setResourceCategory(ResourceCategory.dragabledata);
        resource2.setLabel("res_test1");
        interventionContainer = container.addResource(interventionContainer.getId(), resource2);

        resource = interventionContainer.getResources().get(0);

        Assert.assertEquals("res_test", resource.getLabel());
        Assert.assertEquals(ResourceCategory.dragabledata, resource.getResourceCategory());
    }

    @Test
    public void testPlaceResource(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);

        Resource resource = new Resource();
        resource.setResourceCategory(ResourceCategory.dragabledata);
        resource.setLabel("res_test");
        interventionContainer = container.addResource(interventionContainer.getId(), resource);

        resource = interventionContainer.getResources().get(0);

        resource.setState(State.planned);
        interventionContainer = container.placeResource(interventionContainer.getId(), resource);

        Assert.assertEquals("res_test", resource.getLabel());
        Assert.assertEquals(ResourceCategory.dragabledata, resource.getResourceCategory());
        Assert.assertEquals(State.planned, resource.getState());

        Resource resource2 = new Resource();
        resource2.setResourceCategory(ResourceCategory.dragabledata);
        resource2.setLabel("res_test2");
        resource2.setState(State.planned);
        resource2.setStateDate(State.planned);
        interventionContainer = container.placeResource(interventionContainer.getId() ,resource2);

        Assert.assertEquals("res_test2", interventionContainer.getResources().get(1).getLabel());
    }

    @Test
    public void testAddWatchPath(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);
        Path path = new Path();
        path.addPosition(new Position(48.0, -1.6));
        interventionContainer = container.addWatchPath(interventionContainer.getId(), path);

        Path path2 = new Path();
        path2.addPosition(new Position(48.0, -1.6));
        interventionContainer = container.addWatchPath(interventionContainer.getId(), path2);

        Position position = interventionContainer.getWatchPath().get(0).getPositions().get(0);

        Assert.assertEquals(48.0, position.getLatitude(), 0);
        Assert.assertEquals(-1.6, position.getLongitude(), 0);
    }

    @Test
    public void testUpdateWatchPath(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);
        Path path = new Path();
        Position position = new Position(48.0, -1.6);
        path.addPosition(position);

        interventionContainer = container.addWatchPath(interventionContainer.getId(), path);

        path.addPosition(position);
        interventionContainer = container.updateWatchPath(interventionContainer.getId(), path);

        Assert.assertEquals(2, interventionContainer.getWatchPath().get(0).getPositions().size());

        Path path2 = new Path();
        Intervention intervention = container.updateWatchPath(interventionContainer.getId(), path2);

        Assert.assertEquals(interventionContainer, intervention);
    }

    @Test
    public void testDeleteWatchPath() {
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6), true);
        Path path = new Path();
        Position position = new Position(48.0, -1.6);
        path.addPosition(position);

        interventionContainer = container.addWatchPath(interventionContainer.getId(), path);

        interventionContainer = container.deleteWatchPath(interventionContainer.getId(), path);

        Assert.assertEquals(0, interventionContainer.getWatchPath().size());

        Path path2 = new Path();
        Intervention intervention = container.deleteWatchPath(interventionContainer.getId(), path2);

        Assert.assertEquals(interventionContainer, intervention);
    }

    @Test
    public void testGetInterventionById() {
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test2", 2, 48.0, -1.6), true);
        Intervention intervention = container.getInterventionById(interventionContainer.getId());

        Assert.assertEquals(interventionContainer.getId(), intervention.getId());
        Assert.assertEquals(interventionContainer.getName(), intervention.getName());
        Assert.assertEquals(interventionContainer.getLatitude(), intervention.getLatitude(), 0);
        Assert.assertEquals(interventionContainer.getLongitude(), intervention.getLongitude(), 0);
        Assert.assertEquals(interventionContainer.getIncidentCode(), intervention.getIncidentCode());
    }

    @Test
    public void testGetLastUpdate() {
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test2", 2, 48.0, -1.6), true);

        Assert.assertNotEquals(null, container.getLastUpdate(interventionContainer.getId()));
        Assert.assertEquals(null, container.getLastUpdate(10));
    }

    @Test
    public void testGetNewerLastUpdate() {
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test2", 2, 48.0, -1.6), true);

        Assert.assertEquals(interventionContainer.getLastUpdate(), container.getNewerLastUpdate());
    }
}
