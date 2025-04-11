#include <iostream>
#include <cstring>
#include <MQTTAsync.h>
#include <chrono>
#include <thread>
// Broker and client settings
const std::string ADDRESS = "tcp://broker.hivemq.com:1883";
const std::string CLIENTID = "ExampleClientID";
const std::string TOPIC = "MQTTExamples";
const std::string PAYLOAD = "Hello World!";
const int QOS = 1;
const int TIMEOUT = 10000L;

void onConnect(void* context, MQTTAsync_successData* response) {
    std::cout << "Connected successfully!" << std::endl;
}

void onConnectFailure(void* context, MQTTAsync_failureData* response) {
    // std::cout << "Connect failed, rc " << response ? response->code : 0 << std::endl;
      std::cout << "Connect failed, rc " << response->code << std::endl;
}

int main() {
    MQTTAsync client;
    MQTTAsync_createOptions create_opts = MQTTAsync_createOptions_initializer;
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
   
   create_opts.struct_version = 1;
   create_opts.MQTTVersion = MQTTVERSION_5;
    int rc;

    // Create the client
    rc = MQTTAsync_createWithOptions(&client, ADDRESS.c_str(), CLIENTID.c_str(), MQTTCLIENT_PERSISTENCE_NONE, NULL, &create_opts);
    if (rc != MQTTASYNC_SUCCESS) {
        std::cerr << "Failed to create client, return code " << rc << std::endl;
        return EXIT_FAILURE;
    }

    // Set connect options
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.onSuccess = onConnect;
    conn_opts.onFailure = onConnectFailure;
    conn_opts.context = client;

    // Connect to the broker
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS) {
        std::cerr << "Failed to start connect, return code " << rc << std::endl;
        return EXIT_FAILURE;
    }

    // Wait for the connection to complete
    while (!MQTTAsync_isConnected(client)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Publish a message
    MQTTAsync_responseOptions pub_opts = MQTTAsync_responseOptions_initializer;
    MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
    pubmsg.payload = (void*)PAYLOAD.c_str();
    pubmsg.payloadlen = (int)PAYLOAD.size();
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    if ((rc = MQTTAsync_sendMessage(client, TOPIC.c_str(), &pubmsg, &pub_opts)) != MQTTASYNC_SUCCESS) {
        std::cerr << "Failed to publish message, return code " << rc << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Message published!" << std::endl;

    // Disconnect from the broker
    MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
    if ((rc = MQTTAsync_disconnect(client, &disc_opts)) != MQTTASYNC_SUCCESS) {
        std::cerr << "Failed to start disconnect, return code " << rc << std::endl;
        return EXIT_FAILURE;
    }

    // Cleanup
    MQTTAsync_destroy(&client);
    std::cout << "Disconnected and cleaned up!" << std::endl;

    return EXIT_SUCCESS;
}