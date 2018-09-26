package main

import (
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"time"

	MQTT "github.com/eclipse/paho.mqtt.golang"
	"github.com/google/uuid"
)

var (
	subTopics = []string{
		"/cooktroller/charles/smoker/set_temp",
		"/cooktroller/charles/smoker/runstate",
	}
	serverState ServerState
)

//Server state and tuning parameters
type ServerState struct {
	KP      float32
	KI      float32
	KD      float32
	Loop    int16
	Running bool
	TempSet float32
}

//Server status message
type ServerStatus struct {
	DeviceID string  `json:"device_id"`
	Running  bool    `json:"running"`
	TempCel  float32 `json:"temp_cel"`
	TempFar  float32 `json:"temp_far"`
}

//SetTemperature unmarshal json to set temp
type SetTemperature struct {
	TempFar float32 `json:"temp_far"`
	TempCel float32 `json:"temp_cel"`
}

//SetRunningState unmarshal json to set run state
type SetRunningState struct {
	Running bool `json:"on"`
}

func main() {
	log.Println("Hello world")
	err := connect()
	if err != nil {
		log.Println("application died")
	}
}

//Connect to and setup MQTT
func connect() error {
	cid := uuid.New().String()
	connOpts := MQTT.NewClientOptions()
	connOpts.SetClientID(cid)
	connOpts.SetCleanSession(true)
	connOpts.SetAutoReconnect(true)
	connOpts.SetMaxReconnectInterval(1 * time.Second)
	connOpts.SetTLSConfig(getTLSConfig())
	log.Println("Certs loaded")
	brokerURL := fmt.Sprintf("tcps://%s:%d%s", "a10cp24047duti.iot.us-east-1.amazonaws.com", 8883, "/")
	connOpts.AddBroker(brokerURL)
	mqttClient := MQTT.NewClient(connOpts)
	token := mqttClient.Connect()
	token.WaitTimeout(30 * time.Second)
	token.Wait()

	if token.Error() != nil {
		log.Println(token.Error())
		return token.Error()
	}
	log.Println("MQTT Endpoint connected")
	for _, topic := range subTopics {
		token := mqttClient.Subscribe(topic, 1, subscriber)
		token.WaitTimeout(30 * time.Second)
		token.Wait()
		if token.Error() != nil {
			return token.Error()
		}
		log.Println("Subscribed to topic: ", topic)
	}
	//Start the PID control loop
	pidControlLoop(10*time.Second, doControl, mqttClient)
	return nil
}

//Function to subscribe to a topic
func subscriber(client MQTT.Client, msg MQTT.Message) {
	log.Println("Message", string(msg.Payload()))
	if msg.Topic() == subTopics[0] {
		var setTemp SetTemperature
		err := json.Unmarshal(msg.Payload(), setTemp)
		if err != nil {
			log.Println("Unable to unmarshal temp set")
			return
		}
		serverState.TempSet = setTemp.TempFar //set the temperature (F for now)
	} else if msg.Topic() == subTopics[1] {
		var setRunState SetRunningState
		err := json.Unmarshal(msg.Payload(), setRunState)
		if err != nil {
			log.Println("Unable to unmarshal set running state")
			return
		}
		serverState.Running = setRunState.Running //set opcode running
	} else {
		log.Println("Message from unknown topic")
	}

}

//Enter a loop to run the PID controller on a schedule
func pidControlLoop(d time.Duration, f func(time.Time), mqttClient MQTT.Client) {
	for x := range time.Tick(d) {
		log.Println("Running control loop")
		go f(x)
		var status ServerStatus
		go status.sendStatus(mqttClient)
	}
}

//TODO: Perform the PID Control
func doControl(t time.Time) {
	log.Println("running control")
}

//Set the status on the struct and send it to the MQTT backend
func (status *ServerStatus) sendStatus(mqttClient MQTT.Client) {
	status.DeviceID = "Cooktroller Smoker"
	status.Running = serverState.Running
	status.TempFar = serverState.TempSet
	data, err := json.Marshal(status)
	if err != nil {
		log.Println(err)
	}
	token := mqttClient.Publish("/cooktroller/charles/smoker/status", 1, false, data)
	token.WaitTimeout(30 * time.Second)
	token.Wait()
	if token.Error() != nil {
		log.Println("Unable to send message to MQTT", token.Error())
	}
}

//NewTlsConfig Load in the certificates and setup the TLS configurations and certs
func getTLSConfig() *tls.Config {
	// Import trusted certificates from CAfile.pem.
	// Alternatively, manually add CA certificates to
	// default openssl CA bundle.
	certpool := x509.NewCertPool()
	pemCerts, err := ioutil.ReadFile("/etc/cooktroller/ca.cert")
	if err == nil {
		certpool.AppendCertsFromPEM(pemCerts)
	}

	// Import client certificate/key pair
	cert, err := tls.LoadX509KeyPair("/etc/cooktroller/cert.pem", "/etc/cooktroller/key.pem")
	if err != nil {
		fmt.Println("Could not load X509 Key pair")
		return nil
	}

	// Just to print out the client certificate..
	cert.Leaf, err = x509.ParseCertificate(cert.Certificate[0])
	if err != nil {
		panic(err)
	}
	//log.Println(cert.Leaf)

	// Create tls.Config with desired tls properties
	return &tls.Config{
		// RootCAs = certs used to verify server cert.
		RootCAs: certpool,
		// ClientAuth = whether to request cert from server.
		// Since the server is set up for SSL, this happens
		// anyways.
		ClientAuth: tls.NoClientCert,
		// ClientCAs = certs used to validate client cert.
		ClientCAs: nil,
		// InsecureSkipVerify = verify that cert contents
		// match server. IP matches what is in cert etc.
		InsecureSkipVerify: true,
		// Certificates = list of certs client sends to server.
		Certificates: []tls.Certificate{cert},
	}
}
