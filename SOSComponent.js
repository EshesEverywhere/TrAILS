import React, { useState, useEffect } from "react";
import mqtt from "mqtt";
import "./SOSComponent.css";

const mqttHost = "ec2-52-23-7-21.compute-1.amazonaws.com";
const protocol = "mqtt";
const port = "50002";
const topic = "sos";

function formatPhoneNumber(phoneNumber) {
  const cleaned = ("" + phoneNumber).replace(/\D/g, "");
  const match = cleaned.match(/^(\d{3})(\d{3})(\d{4})$/);
  if (match) {
    return "+1 (" + match[1] + ")-" + match[2] + "-" + match[3];
  }
  return phoneNumber;
}

const normalizePhoneNumber = (phoneNumber) => {
  return phoneNumber.replace(/\D/g, "");
};

function SOSComponent() {
  const [client, setClient] = useState(null);
  const [phoneNumberList, setPhoneNumberList] = useState([]);
  const [emailList, setEmailList] = useState([]);
  const [newPhoneNumber, setNewPhoneNumber] = useState("");
  const [newEmail, setNewEmail] = useState("");
  const [errorMessage, setErrorMessage] = useState("");
  const [sosTriggered, setSosTriggered] = useState(false);

  const hostURL = `${protocol}://${mqttHost}:${port}`;

  useEffect(() => {
    const options = {
      keepalive: 60,
      protocolId: "MQTT",
      protocolVersion: 4,
      clean: true,
      reconnectPeriod: 1000,
      connectTimeout: 30 * 1000,
    };

    const mqttClient = mqtt.connect(hostURL, options);
    setClient(mqttClient);

    mqttClient.on("connect", () => {
      console.log("Connected to MQTT broker");
      mqttClient.subscribe(topic, (err) => {
        if (!err) {
          console.log(`Subscribed to topic: ${topic}`);
        }
      });
    });

    mqttClient.on("message", (topic, message) => {
      if (topic === "sos") {
        if (message.toString() === "O") {
          console.log("SOS triggered");
          setSosTriggered(true); // Set SOS triggered state
          sendTextMessages(); // Send text messages when SOS triggered
        } else if (message.toString() === "F") {
          console.log("SOS turned off");
          setSosTriggered(false); // Set SOS triggered state
        }
      }
    });

    // Load phone numbers from localStorage when component mounts
    const storedPhoneNumbers = JSON.parse(localStorage.getItem("phoneNumbers"));
    if (storedPhoneNumbers) {
      setPhoneNumberList(storedPhoneNumbers);
    }

    // Load email addresses from localStorage when component mounts
    const storedEmails = JSON.parse(localStorage.getItem("emails"));
    if (storedEmails) {
      setEmailList(storedEmails);
    }

    // Load SOS status from localStorage when component mounts
    const storedSosStatus = localStorage.getItem("sosStatus");
    if (storedSosStatus) {
      setSosTriggered(storedSosStatus === "true");
    }

    return () => {
      if (mqttClient) {
        mqttClient.end();
      }
    };
  }, []);

  useEffect(() => {
    // Save phone numbers to localStorage whenever the list changes
    localStorage.setItem("phoneNumbers", JSON.stringify(phoneNumberList));
  }, [phoneNumberList]);

  useEffect(() => {
    // Save email addresses to localStorage whenever the list changes
    localStorage.setItem("emails", JSON.stringify(emailList));
  }, [emailList]);

  useEffect(() => {
    // Save SOS status to localStorage whenever it changes
    localStorage.setItem("sosStatus", sosTriggered.toString());
  }, [sosTriggered]);

  const sendTextMessages = () => {
    const storedPhoneNumbers = JSON.parse(localStorage.getItem("phoneNumbers"));
    const options = {
      keepalive: 60,
      protocolId: "MQTT",
      protocolVersion: 4,
      clean: true,
      reconnectPeriod: 1000,
      connectTimeout: 30 * 1000,
    };

    const mqttClient = mqtt.connect(hostURL, options);
    console.log(phoneNumberList);
    storedPhoneNumbers.forEach((phoneNumber) => {
      console.log(phoneNumber);
      mqttClient.publish("text", "+1" + phoneNumber.replace(/\D/g, ""));
    });
  };

  const handleAddPhoneNumber = () => {
    let normalizedPhoneNumber = normalizePhoneNumber(newPhoneNumber.trim());

    if (
      !/^(\d{10}|(\d{3})-(\d{3})-(\d{4})|\(\d{3}\)-\d{3}-\d{4})$/.test(
        normalizedPhoneNumber
      )
    ) {
      setErrorMessage(
        "Please enter a valid phone number in one of the following formats: 0000000000, 000-000-0000, (000)-000-0000."
      );
      return;
    }

    if (phoneNumberList.includes(normalizedPhoneNumber)) {
      setErrorMessage(
        "Phone number already exists. Please enter a different number."
      );
      return;
    }

    setPhoneNumberList([...phoneNumberList, normalizedPhoneNumber]);
    setNewPhoneNumber("");
    setErrorMessage("");
  };

  const handleRemovePhoneNumber = (phoneNumber) => {
    setPhoneNumberList(
      phoneNumberList.filter((number) => number !== phoneNumber)
    );
  };

  const normalizeEmail = (email) => {
    return email.trim().toLowerCase();
  };

  const handleAddEmail = () => {
    let normalizedEmail = normalizeEmail(newEmail.trim());

    if (!/^[\w-]+(\.[\w-]+)*@([\w-]+\.)+[a-zA-Z]{2,7}$/.test(normalizedEmail)) {
      setErrorMessage("Please enter a valid email address.");
      return;
    }

    if (emailList.includes(normalizedEmail)) {
      setErrorMessage(
        "Email address already exists. Please enter a different email."
      );
      return;
    }

    setEmailList([...emailList, normalizedEmail]);
    setNewEmail("");
    setErrorMessage("");
  };

  const handleRemoveEmail = (email) => {
    setEmailList(emailList.filter((emailItem) => emailItem !== email));
  };

  return (
    <div>
      <h2>SOS Contacts</h2>
      <div>
        {phoneNumberList.length === 0 && (
          <p>Emergency phone numbers will be displayed here</p>
        )}
        {phoneNumberList.length > 0 && (
          <div>
            <h3>Phone Numbers</h3>
            <ul>
              {phoneNumberList.map((phoneNumber) => (
                <li key={phoneNumber}>
                  {formatPhoneNumber(phoneNumber)}
                  <button onClick={() => handleRemovePhoneNumber(phoneNumber)}>
                    Remove
                  </button>
                </li>
              ))}
            </ul>
          </div>
        )}
        <input
          type="text"
          value={newPhoneNumber}
          onChange={(e) => setNewPhoneNumber(e.target.value)}
          placeholder="Enter new phone number"
        />
        <button onClick={handleAddPhoneNumber}>Add Phone Number</button>
      </div>
      <div>
        {emailList.length === 0 && (
          <p>Email addresses will be displayed here</p>
        )}
        {emailList.length > 0 && (
          <div>
            <h3>Email Addresses</h3>
            <ul>
              {emailList.map((email) => (
                <li key={email}>
                  {email}
                  <button onClick={() => handleRemoveEmail(email)}>
                    Remove
                  </button>
                </li>
              ))}
            </ul>
          </div>
        )}
        <input
          type="email"
          value={newEmail}
          onChange={(e) => setNewEmail(e.target.value)}
          placeholder="Enter new email address"
        />
        <button onClick={handleAddEmail}>Add Email Address</button>
        {errorMessage && <p style={{ color: "red" }}>{errorMessage}</p>}
      </div>
      {sosTriggered && (
        <p style={{ fontSize: "20px", color: "red" }}>
          SOS has been triggered!
        </p>
      )}
    </div>
  );
}

export default SOSComponent;
