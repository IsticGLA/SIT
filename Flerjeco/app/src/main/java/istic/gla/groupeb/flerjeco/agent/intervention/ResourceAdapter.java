package istic.gla.groupeb.flerjeco.agent.intervention;

import android.content.Context;
import android.graphics.drawable.Drawable;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.TextView;

import java.util.List;

import entity.Resource;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;

    private static class ViewHolder {
        public TextView textViewLabelResource;
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
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_request,parent,false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.textViewLabelResource = (TextView) convertView.findViewById(R.id.textViewLabelResource);
            viewHolder.imageViewStateResource = (ImageView) convertView.findViewById(R.id.imageViewStateResource);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        Resource resource = resources.get(position);

        viewHolder.textViewLabelResource.setText(resource.getLabel());

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
