package dao;

import entity.IncidentCode;
import junit.framework.TestCase;
import org.junit.Before;
import org.junit.BeforeClass;
import org.junit.Test;

/**
 * Created by vivien on 09/04/15.
 */
public class IncidentCodeDAOTest {

    private static IncidentCode incidentCode;

    @BeforeClass
    public static void init() {
        incidentCode = new IncidentCode();
    }

    @Test
    public void createTest() {

    }
}
