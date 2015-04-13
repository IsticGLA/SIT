package istic.gla.groupeb.flerjeco.codis.intervention;

import java.util.List;

import entity.ResourceType;

/**
 * Created by amhachi on 13/04/15.
 */
public interface OnTaskCompleted {
    void onTaskCompleted(List<ResourceType> resources);
}
