package istic.gla.groupeb.flerjeco.springRest;

import android.content.Context;

import java.util.List;

import istic.gla.groupb.nivimoju.entity.Intervention;
import istic.gla.groupb.nivimoju.entity.ResourceType;

/**
 * Created by corentin on 16/04/15.
 */
public interface IResourceTypeLabelsActivity {
    void updateResourceTypeLabels(ResourceType[] resourceTypes);

    Context getContext();
}
