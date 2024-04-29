import React, { useEffect } from 'react';
import './Map.css';
import * as L from "leaflet";
import mqtt from "mqtt";

const mqttHost = "ec2-52-23-7-21.compute-1.amazonaws.com";
const protocol = "mqtt";
const port = "50002";
const topic = "location"; // Change the topic to "location"

function Map() {
    const clientId = "client" + Math.random().toString(36).substring(7);
    const initialLocation = [41.50416, -81.60845];
    const hostURL = `${protocol}://${mqttHost}:${port}`;

    // Retrieve last received location from local storage
    const lastLocation = JSON.parse(localStorage.getItem('lastLocation'));

    // Declare map as a constant
    const map = React.useRef(null);

    const options = {
        keepalive: 60,
        clientId: clientId,
        protocolId: "MQTT",
        protocolVersion: 4,
        clean: true,
        reconnectPeriod: 1000,
        connectTimeout: 30 * 1000,
    };

    let client;

    useEffect(() => {
        // Initialize the map
        map.current = L.map('map').setView(initialLocation, 16);

        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            maxZoom: 19,
            attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        }).addTo(map.current);

        // Add marker if last location exists
        if (lastLocation) {
            const { lat, lng, time } = lastLocation;

            L.marker([lat, lng]).addTo(map.current)
                .bindPopup(`Time: ${time}`)
                .openPopup();

            // Set the center of the map to the last received coordinates
            map.current.setView([lat, lng], 16);
        }

        // MQTT connection setup
        client = mqtt.connect(hostURL, options);

        client.on("error", (err) => {
            console.log("Error: ", err);
            client.end();
        });

        client.on("reconnect", () => {
            console.log("Reconnecting...");
        });

        client.on("connect", () => {
            console.log("Client connected: " + clientId);
            mqttSub({ topic, qos: 0 }); // Subscribe to the "location" topic
        });

        client.on("message", (topic, message, packet) => {
            const location = message.toString().split(",");
            const lat = parseFloat(location[1].trim()); // Parse latitude
            const lng = parseFloat(location[2].trim()); // Parse longitude
            const time = new Date().toLocaleString(); // Set time to current time
            
            // Store last received latitude, longitude, and time in local storage
            localStorage.setItem('lastLocation', JSON.stringify({ lat, lng, time }));

            console.log("Received Latitude:", lat);
            console.log("Received Longitude:", lng);

            // Clear existing markers
            map.current.eachLayer((layer) => {
                if (layer instanceof L.Marker) {
                    map.current.removeLayer(layer);
                }
            });

            // Add marker with popup
            L.marker([lat, lng]).addTo(map.current)
                .bindPopup(`Time: ${time}`)
                .openPopup();

            // Set the center of the map to the received coordinates
            map.current.setView([lat, lng], 16);
        });

        return () => {
            if (client) {
                client.end();
            }
        };
    }, []);

    const mqttSub = (subscription) => {
        if (client) {
            const { topic, qos } = subscription;
            client.subscribe(topic, { qos }, (error) => {
                if (error) {
                    console.log('Subscribe to topics error', error);
                    return;
                }
                console.log("Subscribed to topic: " + topic);
            });
        }
    };

    return (
        <div id="map"></div>
    );
}

export default Map;
