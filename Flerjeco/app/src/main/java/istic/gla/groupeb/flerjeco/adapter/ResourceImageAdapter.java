package istic.gla.groupeb.flerjeco.adapter;

import android.content.Context;
import android.graphics.Bitmap;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ArrayAdapter;
import android.widget.ImageView;

import java.util.List;

import istic.gla.groupb.nivimoju.entity.Resource;
import istic.gla.groupeb.flerjeco.R;

/**
 * Created by flolegazier on 13/04/15.
 */
public class ResourceImageAdapter extends ArrayAdapter<Resource> {

    private List<Resource> resources;
    private List<Bitmap> bitmaps;

    private static class ViewHolder {
        public ImageView imageViewResource;
    }

    public ResourceImageAdapter(Context context, int resource, List<Resource> resources, List<Bitmap> bitmaps) {
        super(context, resource, resources);
        this.resources = resources;
        this.bitmaps = bitmaps;
    }

    @Override
    public View getView(int position, View convertView, ViewGroup parent){
        ViewHolder viewHolder;
        if (convertView == null){
            convertView = LayoutInflater.from(this.getContext()).inflate(R.layout.item_resource_agent_only_image,parent,false);
            // configure view holder
            viewHolder = new ViewHolder();
            viewHolder.imageViewResource = (ImageView) convertView.findViewById(R.id.image_view);
            convertView.setTag(viewHolder);
        }else{
            viewHolder = (ViewHolder) convertView.getTag();
        }

        viewHolder.imageViewResource.setImageBitmap(bitmaps.get(position));

        return convertView;
    }
}
