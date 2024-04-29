import os
import paho.mqtt.client as mqtt
from twilio.rest import Client

account_sid = "AC57aaf5f4d06bbc0a26b6f7431312198c"
api_key = "SK5e07e631d4edc2e227287d000a9bd031"
api_secret = "[OurSecretKey]"
twilio_client = Client(api_key, api_secret, account_sid)


class TextMessageService(object):
    def __init__(self):
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
        self._client.enable_logger()
        self._client.on_connect = self.on_connect
        self._client.on_message = self.message_received

    def on_connect(self, client, userdata, rc, unknown):
        self._client.subscribe("text", qos=1)

    def message_received(self, client, userdata, message):
        if(message.topic == "text"):
            print(message.payload)
            message = twilio_client.messages.create(
                body='TrAILS SOS has been activated! Go to [TrAILS] website for real time GPS location.',
                messaging_service_sid='MG57da171ec2f6d6f8ccd02e50fe8c6eef',
                to=message.payload)

    def serve(self):
        while True:
            self._client.connect("localhost",
                                 port=50001,
                                 keepalive=1)
            print("Connnected to broker")
            self._client.loop_forever()


if __name__ == '__main__':
    service = TextMessageService().serve()