package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import java.util.List;

import istic.gla.groupb.nivimoju.entity.Image;

/**
 * interface d'une classe utilisant une task de MAJ des image par intervention et position
 */
public interface ImageRefresher {
    void updateWithImages(List<Image> images);
}
