package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;

import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;
import istic.gla.groupeb.flerjeco.icons.Vehicle;
import istic.gla.groupeb.flerjeco.view.IconView;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;

    private static class ViewHolder {
        public IconView iconViewResource;
        public ImageView imageViewStateResource;
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
            viewHolder.imageViewStateResource = (ImageView) convertView.findViewById(R.id.imageViewStateResource);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        Resource resource = resources.get(position);

        Vehicle vehicle = new Vehicle(resource.getLabel());
        viewHolder.iconViewResource.setmVehicle(vehicle);

        Drawable drawable;

        switch (resource.getState()){
            case active:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_online);
                break;
            case planned:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_online);
                break;
            case validated:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_online);
                break;
            case waiting:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_away);
                break;
            case refused:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_busy);
                break;
            default:
                drawable = convertView.getResources().getDrawable(android.R.drawable.presence_away);
                break;
        }

        viewHolder.imageViewStateResource.setImageDrawable(drawable);

        return convertView;
    }
}
