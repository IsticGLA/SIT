package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.CheckBox;

import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;
import util.ResourceRole;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;

    private static class ViewHolder {
        public IconView iconViewResource;
        public CheckBox checkBoxResource;
    }

    public ResourceAdapter(Context context, int resource, List<Resource> resources) {
        super(context, resource, resources);
        this.resources = resources;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent){
        ViewHolder viewHolder;
        if (convertView == null){
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_resource_agent,parent,false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.iconViewResource = (IconView) convertView.findViewById(R.id.iconViewResource);
            viewHolder.checkBoxResource = (CheckBox) convertView.findViewById(R.id.checkboxResource);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        Resource resource = resources.get(position);

        ResourceRole role = ResourceRole.otherwise;
        if (resource.getResourceRole()!=null) {
            role = resource.getResourceRole();
        }
        Vehicle vehicle = new Vehicle(resource.getLabel(),role,resource.getState());
        viewHolder.iconViewResource.setmVehicle(vehicle);

        return convertView;
    }
}
