package istic.gla.groupb.nivimoju.drone.latlong;

import entity.Position;
import org.apache.log4j.Logger;

/**
 * Classe de convertissage entre un système géodesique (latlong) et des coordonnées dans un repère local
 *
 * L'espace de travail est un rectangle dont les bords suivent les lignes de latitudes et longitudes (pas "en biais")
 */
public class LatLongConverter {
    Logger logger = Logger.getLogger(LatLongConverter.class);

    private final double latitudeTop;
    private final double latitudeBottom;
    private final double longitudeLeft;
    private final double longitudeRight;
    private final double width;
    private final double height;
    private final double offsetX;
    private final double offsetY;

    //utilisé pour annuler la déviation suivant l'écart par rapport au centre (delta/coordonnéecaluléesansfacteur)
    private final static double deltaXFactor = -0.0602682894;
    private final static double deltaYFactor = -0.045858885;


    /**
     * Initialise un convertisseur latlong vers coordonnées locales dans un périmètre donné
     * Le point de coordonnées locales (0,0) est considéré comme étant au centre du rectangle déterminé par les latitudes et longitudes renseignées
     *
     * @param latitudeTop le bord supérieur du périmètre de travail
     * @param longitudeLeft le bord gauche du périmètre de travail
     * @param latitudeBottom le bord inférieur du périmètre de travail
     * @param longitudeRight le bord droit du périmètre de travail
     * @param width la dimension du bord gauche à droite en unités du repère local
     * @param height la dimension du bord inférieur à supérieur en unités du repère local
     */
    public LatLongConverter(double latitudeTop, double longitudeLeft, double latitudeBottom, double longitudeRight, double width, double height){
        this.latitudeTop = latitudeTop;
        this.longitudeLeft = longitudeLeft;
        this.latitudeBottom = latitudeBottom;
        this.longitudeRight = longitudeRight;
        this.width = width;
        this.height = height;
        this.offsetX = width / 2;
        this.offsetY = height / 2;
    }

    /**
     * Initialise un convertisseur latlong vers coordonnées locales dans un périmètre donné
     * Le point de coordonnée s locales (0,0) est considéré comme étant au centre du rectangle déterminé par les latitudes et longitudes renseignées
     *
     * @param topLeft le point en haut a gauche du périmètre de travail en coordonnées latlong
     * @param bottomRight le point en bas a droite du périmètre de travail en coordonnées latlong
     * @param width la dimension du bord gauche à droite en unités du repère local
     * @param height la dimension du bord inférieur à supérieur en unités du repère local
     */
    public LatLongConverter(Position topLeft, Position bottomRight, double width, double height){
        this(topLeft.getLatitude(), topLeft.getLongitude(), bottomRight.getLatitude(), bottomRight.getLongitude(), width, height);
    }

    /**
     * Retourne les coordonnées locales correspondantes à une coordonnée latlong
     * @param latlong les coordonnées à transformer
     * @return les coordonnées dans le système local
     * @throws java.lang.IllegalArgumentException si les coordonnées demandées sont en dehors du périmètre de travail
     */
    public LocalCoordinate getLocal(Position latlong) throws IllegalArgumentException{
        if(latlong.getLatitude() < latitudeBottom || latlong.getLatitude() > latitudeTop
                || latlong.getLongitude() < longitudeLeft ||latlong.getLongitude() > longitudeRight){
            logger.error("Les coordonnées sont en dehors du périmètre de travail, conversion impossible");
            throw  new IllegalArgumentException("Impossible de convertir une position hors du périmètre de travail défini");
        }
        //interpolation linéaire sur x puis y
        double ratioX = (latlong.getLongitude() - longitudeLeft) / (longitudeRight - longitudeLeft);
        double x = width * ratioX - offsetX;
        double ratioY = (latlong.getLatitude() - latitudeBottom) / (latitudeTop - latitudeBottom);
        double y = height * ratioY - offsetY;

        //prise en compte de la déviation
        x = x*(1+deltaXFactor);
        y = y*(1+deltaYFactor);

        logger.debug(String.format("width:%s, ratioX:%s, offsetX:%s, x=>%s", width, ratioX, offsetX, x));
        logger.debug(String.format("height:%s, ratioY:%s, offsetY:%s, y=>%s", height, ratioY, offsetY, y));

        return new LocalCoordinate(x, y, 0);
    }


    /**
     * Retourne les coordonnées latlong correspondantes à une coordonnée locale
     * @param local les coordonnées à transformer
     * @return les coordonnées dans le système latlong
     */
    public Position getLatLong(LocalCoordinate local){
        //interpolation linéaire sur x puis y
        double maxX = width - offsetX;
        double maxY = height - offsetY;
        double longitude = (longitudeRight - longitudeLeft) * (local.getX() - maxX) / (maxX - offsetX) + longitudeLeft;
        double latitude = (latitudeTop - latitudeBottom) * (local.getY() - maxY) / (maxY - offsetY) + latitudeBottom;


        return new Position(longitude,  latitude);
    }

    @Override
    public String toString() {
        return "LatLongConverter{" +
                "latitudeTop=" + latitudeTop +
                ", latitudeBottom=" + latitudeBottom +
                ", longitudeLeft=" + longitudeLeft +
                ", longitudeRight=" + longitudeRight +
                ", width=" + width +
                ", height=" + height +
                '}';
    }
}
