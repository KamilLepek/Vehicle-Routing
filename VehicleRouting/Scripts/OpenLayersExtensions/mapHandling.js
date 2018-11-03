//function initMap() { //TODO: move this to function instead of explicitly invoking script in div?
    const map = new ol.Map({
        target: 'map',
        layers: [
            new ol.layer.Tile({
                source: new ol.source.OSM()
            })
        ],
        view: new ol.View({
            center: ol.proj.fromLonLat([18.618, 54.373]),
            zoom: 16
        })
    });
//}