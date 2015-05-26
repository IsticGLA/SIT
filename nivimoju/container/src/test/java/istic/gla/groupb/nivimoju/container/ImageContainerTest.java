package istic.gla.groupb.nivimoju.container;

import istic.gla.groupb.nivimoju.entity.Image;
import org.junit.Before;
import org.junit.Test;

import java.sql.Timestamp;

import static org.junit.Assert.assertEquals;

/**
 * Created by jules on 26/05/15.
 */
public class ImageContainerTest {

    private static final String DRONE = "drone_1";
    private static final String IMAGE = "image";
    private ImageContainer imageContainer;
    private Image image;


    @Before
    public void init() {
        imageContainer = ImageContainer.getInstance();
        image = new Image(new Timestamp(0).getTime(), new double[]{0, 0}, IMAGE, 0);
    }

    @Test
    public void testSetLastImageOfDrone() {
        imageContainer.setLastImageOfDrone(DRONE, image);
    }

    @Test
    public void testGetLastImageOfDrone() {
        imageContainer.setLastImageOfDrone(DRONE, image);
        assertEquals(image, imageContainer.getLastImageOfDrone(DRONE));
    }
}
