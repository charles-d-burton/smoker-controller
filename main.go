package main

import (
	"crypto/tls"
	"crypto/x509"
	"io/ioutil"
	"log"
	"fmt"
)

func main() {
	log.Println("Hello world")
}

func connect() {
  cid := uuid.New().String()
  connOpts := MQTT.NewClientOptions()
  connOpts.SetClientID(cid)
  connOpts.SetCleanSession(true)
  connOpts.SetAutoReconnect(true)
  connOpts.SetMaxReconnectInterval(1* time.Second)
  connOpts.SetTLSConfig(getTLSConfig())
  #mqttClient, err := 
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
