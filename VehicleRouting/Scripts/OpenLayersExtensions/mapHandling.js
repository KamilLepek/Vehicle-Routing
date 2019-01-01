﻿function initMap(vehicles, pointsOfDelivery) {

    var pointOfInterestIconStyle = new ol.style.Style({
        image: new ol.style.Icon(({
            scale: 0.2,
            anchor: [0.0, 1.0],
            anchorXUnits: 'fraction',
            anchorYUnits: 'fraction',
            src: "/Content/Images/pod-map-marker.png"
        }))
    });

    var vehicleIconStyle = new ol.style.Style({
        image: new ol.style.Icon(({
            scale: 0.2,
            anchor: [0.3, 0.5],
            anchorXUnits: 'fraction',
            anchorYUnits: 'fraction',
            src: "/Content/Images/Car1.png"
        }))
    });

    var iconFeaturesList = [];
    for (var i = 0; i < vehicles.length; i++) {
        iconFeaturesList.push(createIconFeature(vehicles[i], vehicleIconStyle));
    }
    for (var i = 0; i < pointsOfDelivery.length; i++) {
        iconFeaturesList.push(createIconFeature(pointsOfDelivery[i], pointOfInterestIconStyle));
    }

    var vectorSource = new ol.source.Vector({
        features: iconFeaturesList
    });

    var vectorLayer = new ol.layer.Vector({
        source: vectorSource
    });
    
    var map = new ol.Map({
        target: 'map',
        layers: [
            new ol.layer.Tile({
                source: new ol.source.OSM()
            }),
            vectorLayer
        ],
        controls: ol.control.defaults().extend([
            new ol.control.FullScreen(),
            new ol.control.Zoom(),
            new ol.control.Attribution(),
            new ol.control.OverviewMap()
        ]),
        view: new ol.View({
            center: ol.proj.fromLonLat([-73.982, 40.748]),
            zoom: 16
        })
    });

    var popup = new ol.Overlay.Popup();
    map.addOverlay(popup);

    map.on('singleclick', function (evt) {
        var lonLat = ol.proj.transform(evt.coordinate, 'EPSG:3857', 'EPSG:4326');
        var lon = ((lonLat[0] + 180) % 360) - 180;
        var lat = lonLat[1];
        popup.show(evt.coordinate, '<div>'
            + '<p>' + lon.toFixed(4) + ',' + lat.toFixed(4) + '</p>'
            + '<p><a href="#" data-target="/Vehicles" data-lat="' + lat.toFixed(4) + '" data-lon="' + lon.toFixed(4) + '">Add vehicle here</a></p>'
            + '<p><a href="#" data-target="/PointsOfDeliveries" data-lat="' + lat.toFixed(4) + '" data-lon="' + lon.toFixed(4) + '">Add point of delivery here</a></p>'
            + '</div>');
    });

    popup.getElement().addEventListener('click', function (e) {
        var target = e.target.getAttribute('data-target');
        var lat = e.target.getAttribute('data-lat').replace(".", "%2E");
        var lon = e.target.getAttribute('data-lon').replace(".", "%2E");
        var url = target + '/Create?lon=' + lon + '&lat=' + lat;
        window.location.replace(url);
    }, false);

    return map;
}

function createIconFeature(localisation, iconStyle) {
    var iconFeature = new ol.Feature({
        geometry: new ol.geom.Point(ol.proj.fromLonLat(localisation))
    });
    iconFeature.setStyle(iconStyle);
    return iconFeature;
}

function showResults(vehicles, pointsOfDelivery, results) {
    var map = initMap(vehicles, pointsOfDelivery);
    for (var i = 0; i < results.length; i++) {
        var randomColor = '#' + (Math.random() * 0xFFFFFF << 0).toString(16);
        var points = getIntermediatePoints(results[i]);
        var lineLayer = getLineLayer(points, randomColor);
        map.addLayer(lineLayer);
    }
}

function getPointsBetweenTwoPoints(pointA, pointB) {
    var pts = [];

    var xmlHttp = new XMLHttpRequest();
    xmlHttp.open("GET", "https://router.project-osrm.org/route/v1/driving/" + pointA[0] + ',' + pointA[1] + ';' + pointB[0] + ',' + pointB[1] + "?steps=true&geometries=polyline&overview=false", false);
    xmlHttp.send(null);
    var jsonResponse = JSON.parse(xmlHttp.responseText);

    var points = jsonResponse.routes[0].legs[0].steps;

    for (var i = 0; i < points.length; i++) {
        for (var j = 0; j < points[i].intersections.length; j++) {
            pts.push(points[i].intersections[j].location);
        }
    }

    return pts;
}

function getIntermediatePoints(points) {
    var pts = [];
    for (var i = 0; i < points.length; i++) {
        pts.push(points[i]);
        if (i + 1 < points.length) {          
            pts.push.apply(pts, getPointsBetweenTwoPoints(points[i], points[i + 1]));
        } 
    }
    return pts;
}

function getLineLayer(points, col) {
    for (var i = 0; i < points.length; i++) {
        points[i] = ol.proj.transform(points[i], 'EPSG:4326', 'EPSG:3857');
    }

    var featureLine = new ol.Feature({
        geometry: new ol.geom.LineString(points)
    });

    var vectorLine = new ol.source.Vector({});
    vectorLine.addFeature(featureLine);

    var vectorLineLayer = new ol.layer.Vector({
        source: vectorLine,
        style: new ol.style.Style({
            fill: new ol.style.Fill({ color: col, weight: 6 }),
            stroke: new ol.style.Stroke({ color: col, width: 4 })
        })
    });

    return vectorLineLayer;
}