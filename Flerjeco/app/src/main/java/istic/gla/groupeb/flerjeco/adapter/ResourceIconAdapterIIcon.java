package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;

import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.IIcon;
import istic.gla.groupeb.flerjeco.view.IconView;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceIconAdapterIIcon extends ArrayAdapter<Resource> {

    private List<Resource> resources;
    private List<IIcon> icons;

    private static class ViewHolder {
        public IconView iconViewResource;
    }

    public ResourceIconAdapterIIcon(Context context, int resource, List<Resource> resources, List<IIcon> icons) {
        super(context, resource, resources);
        this.resources = resources;
        this.icons = icons;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent){
        ViewHolder viewHolder;
        if (convertView == null){
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_resource_agent_only_icon,parent,false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.iconViewResource = (IconView) convertView.findViewById(R.id.icon_view);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        viewHolder.iconViewResource.setIcon(icons.get(position));
        viewHolder.iconViewResource.setResource(resources.get(position));

        return convertView;
    }
}
