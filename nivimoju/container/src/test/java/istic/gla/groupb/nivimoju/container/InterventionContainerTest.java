package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.Path;
import istic.gla.groupb.nivimoju.entity.Position;
import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupb.nivimoju.util.ResourceCategory;
import istic.gla.groupb.nivimoju.util.State;
import org.apache.log4j.Logger;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

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
        container = InterventionContainer.getInstance(true);
    }

    @Test
    public void testCreateIntervention(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test1", 1, 48.0, -1.6));

        Assert.assertEquals("inter_test1", interventionContainer.getName());
        Assert.assertEquals(48.0, interventionContainer.getLatitude(), 0);
        Assert.assertEquals(-1.6, interventionContainer.getLongitude(), 0);
        Assert.assertEquals(1, interventionContainer.getIncidentCode());
    }

    @Test
    public void testUpdateIntervention(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test2", 2, 48.0, -1.6));

        interventionContainer.setName("inter_test2_updated");
        interventionContainer = container.updateIntervention(interventionContainer);

        Assert.assertEquals("inter_test2_updated", interventionContainer.getName());
    }

    @Test
    public void testChangeResourceState(){
        Intervention intervention = new Intervention("inter_test3", 3, 48.0, -1.6);
        Resource res = new Resource();
        intervention.getResources().add(0, res);
        Intervention interventionContainer = container.createIntervention(intervention);

        interventionContainer = container.changeResourceState(interventionContainer.getId(), 0L, State.arrived.toString());

        Assert.assertEquals(State.arrived, interventionContainer.getResources().get(0).getState());
    }

    @Test
    public void testAddVehicle(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));

        interventionContainer = container.addVehicle(interventionContainer.getId(), "vehicle_test");
        Resource vehicle = interventionContainer.getResources().get(0);

        Assert.assertEquals("vehicle_test" + vehicle.getIdRes(), vehicle.getLabel());
        Assert.assertEquals(ResourceCategory.vehicule, vehicle.getResourceCategory());
    }

    @Test
    public void testAddResource(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));

        Resource resource = new Resource();
        resource.setResourceCategory(ResourceCategory.dragabledata);
        resource.setLabel("res_test");
        interventionContainer = container.addResource(interventionContainer.getId(), resource);

        Assert.assertEquals("res_test", resource.getLabel());
        Assert.assertEquals(ResourceCategory.dragabledata, resource.getResourceCategory());
    }

    @Test
    public void testPlaceResource(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));

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
    }

    @Test
    public void testAddWatchPath(){
        Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));
        Path path = new Path();
        path.addPosition(new Position(48.0, -1.6));
        interventionContainer = container.addWatchPath(interventionContainer.getId(), path);

        Position position = interventionContainer.getWatchPath().get(0).getPositions().get(0);

        Assert.assertEquals(48.0, position.getLatitude(), 0);
        Assert.assertEquals(-1.6, position.getLongitude(), 0);
    }

    @Test
    public void testUpdateWatchPath(){Intervention interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));
        Path path = new Path();
        path.addPosition(new Position(48.0, -1.6));
        interventionContainer = container.addWatchPath(interventionContainer.getId(), path);

        Position position = interventionContainer.getWatchPath().get(0).getPositions().get(0);
         interventionContainer = container.createIntervention(new Intervention("inter_test4", 3, 48.0, -1.6));
        Path path2 = new Path();
        path2.addPosition(new Position(48.0, -1.6));
        interventionContainer = container.addWatchPath(interventionContainer.getId(), path2);

        container.updateIntervention(interventionContainer);

        position = interventionContainer.getWatchPath().get(0).getPositions().get(0);


    }
}
