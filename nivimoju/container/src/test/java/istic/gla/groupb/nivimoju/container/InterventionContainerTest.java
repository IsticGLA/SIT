package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.entity.Intervention;
import org.apache.log4j.Logger;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

/**
 * Created by vivien on 26/05/15.
 */
public class InterventionContainerTest {
    Logger logger = Logger.getLogger(DroneContainerTest.class);
    InterventionContainer container;

    @Before
    public void init(){
        DroneContainer.destroy();
        container = InterventionContainer.getInstance();
    }

    @Test
    public void testCreateIntervention(){
        Intervention intervention = new Intervention("inter_test", 1, 48.0, -1.6);
        Intervention interventionContainer = container.createIntervention(intervention);

        Assert.assertEquals("inter_test", interventionContainer.getName());
        Assert.assertEquals(48.0, interventionContainer.getLatitude(), 0);
        Assert.assertEquals(-1.6, interventionContainer.getLongitude(), 0);
        Assert.assertEquals(1, interventionContainer.getIncidentCode());
    }
}
