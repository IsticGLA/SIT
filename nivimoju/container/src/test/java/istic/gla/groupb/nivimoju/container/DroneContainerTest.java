package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.dao.DroneDAO;
import istic.gla.groupb.nivimoju.entity.Drone;
import istic.gla.groupb.nivimoju.entity.Position;
import org.apache.log4j.Logger;
import org.junit.*;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import static org.junit.Assert.*;

public class DroneContainerTest {
    Logger logger = Logger.getLogger(DroneContainerTest.class);
    DroneContainer container;

    @BeforeClass
    public static void initDronesInDb(){
        emptyDatabase();
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connectTest();
        droneDAO.create(new Drone("drone_1", 0, 0, -1));
        droneDAO.create(new Drone("drone_2", 0, 0, -1));
        droneDAO.create(new Drone("drone_3", 0, 0, -1));
        droneDAO.create(new Drone("drone_4", 0, 0, -1));
        droneDAO.create(new Drone("drone_5", 0, 0, -1));
        droneDAO.disconnect();
    }

    @AfterClass
    public static void emptyDatabase(){
        DroneDAO droneDAO = new DroneDAO();
        droneDAO.connectTest();
        for(Drone droneToDelete : droneDAO.getAll()){
            droneDAO.delete(droneToDelete);
        }
        droneDAO.disconnect();
    }

    @Before
    public void init(){
        DroneContainer.destroyTestInstance();
        container = DroneContainer.getTestInstance();
    }

    @Test
    public void testUpdateDronePosition(){
        container.updateDrone("drone_1", new Position(1, 2, 3));
        assertEquals(1L, container.getDroneByLabel("drone_1").getLatitude(), 0);
        assertEquals(2L, container.getDroneByLabel("drone_1").getLongitude(), 0);

        Collection<Drone> drones = container.getDrones();
        Assert.assertEquals(5, drones.size());
        for(Drone drone : drones){
            if(!drone.getLabel().equals("drone_1")){
                assertEquals(0L, drone.getLatitude(), 0);
                assertEquals(0L, drone.getLongitude(), 0);
            }
        }
    }

    @Test
    public void testRequestDrones(){
        Drone drone = container.requestDrone(2L);
        assertNotNull(drone);

        assertEquals(2L, drone.getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get(drone.getLabel()).getIdIntervention());
        assertEquals(1, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(1, container.getDronesAssignedTo(2L).size());
    }

    @Test
    public void testRequestAndFreeDrones(){
        container.requestDrones(2L, 6);

        Collection<Drone> drones = container.getDrones();
        Assert.assertEquals(5, drones.size());

        for(Drone drone : drones){
            assertEquals(2L, drone.getIdIntervention(), 0);
        }

        container.freeDrones(2L, 3);
        drones = container.getDronesAssignedTo(2L);
        Assert.assertEquals(2, drones.size());
    }

    @Test
    public void testRequestDrones2(){
        Drone drone1 = container.requestDrone(2L);
        Drone drone2 = container.requestDrone(2L);
        Drone drone3 = container.requestDrone(2L);
        Drone drone4 = container.requestDrone(2L);
        Drone drone5 = container.requestDrone(2L);
        assertNotNull(drone1);
        assertNotNull(drone2);
        assertNotNull(drone3);
        assertNotNull(drone4);
        assertNotNull(drone5);

        assertEquals(2L, drone1.getIdIntervention());
        assertEquals(2L, drone2.getIdIntervention());
        assertEquals(2L, drone3.getIdIntervention());
        assertEquals(2L, drone4.getIdIntervention());
        assertEquals(2L, drone5.getIdIntervention());

        assertEquals(2L, container.getMapDroneByLabel().get("drone_1").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_2").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_3").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_4").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_5").getIdIntervention());

        assertEquals(5, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(5, container.getDronesAssignedTo(2L).size());

        Drone drone6 = container.requestDrone(2L);
        assertNull(drone6);
        assertEquals(5, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(5, container.getDronesAssignedTo(2L).size());
    }

    @Test
    public void testFreeDrone(){
        Drone drone1 = container.requestDrone(2L);
        Drone drone2 = container.requestDrone(2L);
        Drone drone3 = container.requestDrone(2L);
        Drone drone4 = container.requestDrone(2L);
        Drone drone5 = container.requestDrone(2L);
        ArrayList<Drone> drones = new ArrayList<>();
        drones.add(drone1);
        drones.add(drone2);
        drones.add(drone3);
        drones.add(drone4);
        drones.add(drone5);

        //no change
        assertFalse(container.freeDrone(1L));

        assertEquals(2L, drone1.getIdIntervention());
        assertEquals(2L, drone2.getIdIntervention());
        assertEquals(2L, drone3.getIdIntervention());
        assertEquals(2L, drone4.getIdIntervention());
        assertEquals(2L, drone5.getIdIntervention());

        assertEquals(2L, container.getMapDroneByLabel().get("drone_1").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_2").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_3").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_4").getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get("drone_5").getIdIntervention());

        assertEquals(5, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(5, container.getDronesAssignedTo(2L).size());

        //free a drone
        assertTrue(container.freeDrone(2L));

        assertEquals(4, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(4, container.getDronesAssignedTo(2L).size());
        assertEquals(1, container.getMapDronesByIntervention().get(-1L).size());
        assertEquals(1, container.getDronesAssignedTo(-1L).size());
        Drone freeDrone = container.getMapDronesByIntervention().get(-1L).iterator().next();
        assertNotNull(freeDrone);
        assertEquals(-1L, freeDrone.getIdIntervention());
        logger.info("drone free : " + freeDrone.getLabel());

        for(Drone drone : drones){
            if(drone.getLabel().equals(freeDrone.getLabel())){
                assertEquals(-1L, drone.getIdIntervention());
                assertEquals(-1L, container.getMapDroneByLabel().get(drone.getLabel()).getIdIntervention());
            } else {
                logger.debug("testing drone " + drone.getLabel());
                assertEquals(2L, drone.getIdIntervention());
                assertEquals(2L, container.getMapDroneByLabel().get(drone.getLabel()).getIdIntervention());
            }
        }
    }

    @Test
    public void testFreeDrone2() {
        Drone drone1 = container.requestDrone(2L);
        Drone drone2 = container.requestDrone(2L);
        Drone drone3 = container.requestDrone(2L);
        Drone drone4 = container.requestDrone(2L);
        Drone drone5 = container.requestDrone(2L);

        container.freeDrone(drone1.getLabel());

        assertEquals(-1L, drone1.getIdIntervention());
        assertEquals(2L, drone2.getIdIntervention());
        assertEquals(2L, drone3.getIdIntervention());
        assertEquals(2L, drone4.getIdIntervention());
        assertEquals(2L, drone5.getIdIntervention());

        assertEquals(-1L, container.getMapDroneByLabel().get(drone1.getLabel()).getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get(drone2.getLabel()).getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get(drone3.getLabel()).getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get(drone4.getLabel()).getIdIntervention());
        assertEquals(2L, container.getMapDroneByLabel().get(drone5.getLabel()).getIdIntervention());

        assertEquals(4, container.getMapDronesByIntervention().get(2L).size());
        assertEquals(4, container.getDronesAssignedTo(2L).size());
        assertEquals(1, container.getMapDronesByIntervention().get(-1L).size());
        assertEquals(1, container.getDronesAssignedTo(-1L).size());
    }

    @Test
    public void testRequestAndFree(){
        Drone drone = container.requestDrone(2L);

        assertNotNull(drone);

        assertEquals("le request n'as pas vidé les drones libres", 4, container.getMapDronesByIntervention().get(-1L).size());
        assertEquals("le request n'as pas rempli les drones affectés", 1, container.getMapDronesByIntervention().get(2L).size());
        assertEquals("le request n'as pas modifié le drone", 2L, container.getDroneByLabel(drone.getLabel()).getIdIntervention());

        boolean res = container.freeDrone(2L);
        assertTrue(res);
        assertEquals("le request n'as pas rempli les drones libres", 5, container.getMapDronesByIntervention().get(-1L).size());
        assertNull("le request n'as pas vidé les drones affectés", container.getMapDronesByIntervention().get(2L));
        assertEquals("le request n'as pas modifié le drone", -1L, container.getDroneByLabel(drone.getLabel()).getIdIntervention());

    }
}