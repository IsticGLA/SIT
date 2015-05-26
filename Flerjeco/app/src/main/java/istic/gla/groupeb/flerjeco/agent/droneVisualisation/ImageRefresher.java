package istic.gla.groupeb.flerjeco.agent.droneVisualisation;

import android.content.Context;

import java.util.List;

import istic.gla.groupb.nivimoju.entity.Image;

/**
 * interface d'une classe utilisant une task de MAJ des image par intervention et position
 */
public interface ImageRefresher {
    void loadImages(List<Image> images);
}
