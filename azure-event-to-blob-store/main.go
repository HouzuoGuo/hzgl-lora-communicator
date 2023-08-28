package main

import (
	"bytes"
	"context"
	"encoding/json"
	"errors"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"sync"
	"sync/atomic"
	"time"

	"github.com/Azure/azure-sdk-for-go/sdk/azcore"
	"github.com/Azure/azure-sdk-for-go/sdk/messaging/azeventhubs"
	"github.com/Azure/azure-sdk-for-go/sdk/storage/azblob/appendblob"
	"github.com/Azure/azure-sdk-for-go/sdk/storage/azblob/container"
)

type IotEventMessage struct {
	Version    int                `json:"version"`
	Properties IotEventProperties `json:"properties"`
}

type IotEventProperties struct {
	Reported IotEventReported `json:"reported"`
}

type IotEventReported struct {
	DecodedPayload IotDecodedPayload `json:"decodedPayload"`
}

type IotDecodedPayload struct {
	Altitude                        float64 `json:"altitude"`
	AmbientAltitudeMetre            float64 `json:"ambient_altitude_metre"`
	AmbientHumidityPct              int     `json:"ambient_humidity_pct"`
	AmbientPressureHpa              float64 `json:"ambient_pressure_hpa"`
	AmbientTempCelcius              float64 `json:"ambient_temp_celcius"`
	BattMillivolt                   int     `json:"batt_millivolt"`
	BluetoothLoudestTXMac           string  `json:"bt_loudest_tx_mac"`
	BluetoothLoudestTXRssi          int     `json:"bt_loudest_tx_rssi"`
	BluetoothNumDevices             int     `json:"bt_num_devices"`
	Cpu0ResetReason                 int     `json:"cpu0_reset_reason"`
	Cpu1ResetReason                 int     `json:"cpu1_reset_reason"`
	CpuWakeUpCause                  int     `json:"cpu_wake_up_cause"`
	EspResetReason                  int     `json:"esp_reset_reason"`
	GpsHeadingDeg                   int     `json:"gps_heading_deg"`
	GpsPosAgeSec                    int     `json:"gps_pos_age_sec"`
	GpsSpeedKhm                     int     `json:"gps_speed_kmh"`
	Hdop                            int     `json:"hdop"`
	HeapUsageKB                     int     `json:"heap_usage_kb"`
	IsBattCharging                  int     `json:"is_batt_charging"`
	LastRxSec                       int     `json:"last_rx_sec"`
	Latitude                        float64 `json:"latitude"`
	Longitude                       float64 `json:"longitude"`
	PowerMilliamp                   int     `json:"power_milliamp"`
	Sats                            int     `json:"sats"`
	UptimeSec                       int     `json:"uptime_sec"`
	WifiInflightPktsDataLenAllChans int     `json:"wifi_inflight_pkt_data_len_all_chans"`
	WifiInflightPktsAllChans        int     `json:"wifi_inflight_pkts_all_chans"`
	WifiLoudestTXChan               int     `json:"wifi_loudest_tx_chan"`
	WifiLoudestTXMac                string  `json:"wifi_loudest_tx_mac"`
	WifiLoudestTXRssi               int     `json:"wifi_loudest_tx_rssi"`
}

func (payload *IotDecodedPayload) IsRFSensingTelemetry() bool {
	return payload.Latitude != 0 || payload.Altitude != 0 || payload.Sats > 0 ||
		payload.BluetoothNumDevices > 0 || payload.WifiInflightPktsAllChans > 0
}

func (payload *IotDecodedPayload) IsEnvironmentSensingTelemetry() bool {
	return payload.UptimeSec > 0 || payload.AmbientPressureHpa > 0 || payload.PowerMilliamp > 0
}

type BytesReadCloser struct {
	*bytes.Reader
}

func (a *BytesReadCloser) Close() error {
	return nil
}

func flattenedRecord(evProps map[string]any, env, rf IotDecodedPayload) map[string]any {
	ret := make(map[string]any)
	ret["deviceId"] = evProps["deviceId"]
	ret["hubName"] = evProps["hubName"]
	ret["operationTimestamp"] = evProps["operationTimestamp"]

	ret["ambient_altitude_metre"] = env.AmbientAltitudeMetre
	ret["ambient_humidity_pct"] = env.AmbientHumidityPct
	ret["ambient_pressure_hpa"] = env.AmbientPressureHpa
	ret["ambient_temp_celcius"] = env.AmbientTempCelcius
	ret["batt_millivolt"] = env.BattMillivolt
	ret["cpu0_reset_reason"] = env.Cpu0ResetReason
	ret["cpu1_reset_reason"] = env.Cpu1ResetReason
	ret["cpu_wake_up_cause"] = env.CpuWakeUpCause
	ret["esp_reset_reason"] = env.EspResetReason
	ret["heap_usage_kb"] = env.HeapUsageKB
	ret["is_batt_charging"] = env.IsBattCharging
	ret["last_rx_sec"] = env.LastRxSec
	ret["power_milliamp"] = env.PowerMilliamp
	ret["uptime_sec"] = env.UptimeSec

	ret["altitude"] = rf.Altitude
	ret["bt_loudest_tx_mac"] = rf.BluetoothLoudestTXMac
	ret["bt_loudest_tx_rssi"] = rf.BluetoothLoudestTXRssi
	ret["bt_num_devices"] = rf.BluetoothNumDevices
	ret["gps_heading_deg"] = rf.GpsHeadingDeg
	ret["gps_pos_age_sec"] = rf.GpsPosAgeSec
	ret["gps_speed_kmh"] = rf.GpsSpeedKhm
	ret["hdop"] = rf.Hdop
	ret["latitude"] = rf.Latitude
	ret["longitude"] = rf.Longitude
	ret["sats"] = rf.Sats
	ret["wifi_inflight_pkt_data_len_all_chans"] = rf.WifiInflightPktsDataLenAllChans
	ret["wifi_inflight_pkts_all_chans"] = rf.WifiInflightPktsAllChans
	ret["wifi_loudest_tx_chan"] = rf.WifiLoudestTXChan
	ret["wifi_loudest_tx_mac"] = rf.WifiLoudestTXMac
	ret["wifi_loudest_tx_rssi"] = rf.WifiLoudestTXRssi

	return ret
}

var eventHubConnStr, eventHubConsumerGroup, storageConnStr, storageContainer string

// main continuously reads events coming through the event hub and then simply prints them out.
func main() {
	flag.StringVar(&eventHubConnStr, "eventconnstr", "", "Event hub instance connection string")
	flag.StringVar(&eventHubConsumerGroup, "eventconsumer", "", "Event hub instance consumer group name")
	flag.StringVar(&storageConnStr, "storageconnstr", "", "Storage account shared access signature connection string")
	flag.StringVar(&storageContainer, "storagecontainer", "", "Storage blob container name")
	flag.Parse()
	log.Printf("args: %+#v", os.Args)

	if eventHubConnStr == "" || eventHubConsumerGroup == "" || storageConnStr == "" {
		flag.Usage()
		os.Exit(1)
	}
	for {
		loop()
		time.Sleep(10 * time.Second)
	}
}

func loop() {
	counter := &atomic.Int32{}
	containerClient, err := container.NewClientFromConnectionString(storageConnStr, storageContainer, nil)
	if err != nil {
		log.Panic(err)
	}
	appendBlobClient := initBlobClient(containerClient)

	consumerClient, err := azeventhubs.NewConsumerClientFromConnectionString(eventHubConnStr, "", eventHubConsumerGroup, nil)
	if err != nil {
		log.Panic(err)
	}

	log.Print("getting hub properties")
	properties, err := consumerClient.GetEventHubProperties(context.Background(), nil)
	if err != nil {
		log.Panic(err)
	}
	log.Printf("hub properties: %+#v", properties)

	partClients := make([]*azeventhubs.PartitionClient, 0)
	for _, partitionID := range properties.PartitionIDs {
		partClient, err := consumerClient.NewPartitionClient(partitionID, nil)
		if err != nil {
			log.Panic(err)
		}
		partClients = append(partClients, partClient)
	}

	log.Printf("receiving events from %d partitions", len(partClients))
	wg := new(sync.WaitGroup)
	wg.Add(len(partClients))
	for _, client := range partClients {
		go func(client *azeventhubs.PartitionClient) {
			defer func() {
				timeoutCtx, cancelFunc := context.WithTimeout(context.Background(), 10*time.Second)
				defer cancelFunc()
				log.Printf("close client: %+v", client.Close(timeoutCtx))
			}()
			defer wg.Done()
			for {
				received, err := client.ReceiveEvents(context.Background(), 1, nil)
				if err != nil {
					log.Panic(err)
				}
				for _, ev := range received {
					handleEvent(ev, appendBlobClient)
				}
				if counter.Add(1) == 100 {
					// Start over, re-initialise the clients and the loop.
					log.Printf("re-initialising and then continue")
					return
				}
			}

		}(client)
	}
	wg.Wait()
}

// latestRFSensingPayload is a map of hubName+deviceID to RF sensing frame payload.
var latestRFSensingPayload = make(map[string]IotDecodedPayload)

func handleEvent(ev *azeventhubs.ReceivedEventData, appendBlobClient *appendblob.Client) {
	// Example: map[string]interface {}{"deviceId":"communicator-2", "hubName":"iothub-q7uk544x7yrn6", "iothub-message-schema":"twinChangeNotification", "opType":"updateTwin", "operationTimestamp":"2023-08-27T13:21:20.1471561+00:00"}
	deviceID := fmt.Sprintf("%s", ev.Properties["deviceId"])
	hubName := fmt.Sprintf("%s", ev.Properties["hubName"])
	timestamp := fmt.Sprintf("%s", ev.Properties["operationTimestamp"])
	log.Printf("received event properties %+#v, content: %s", ev.Properties, string(ev.Body))
	if deviceID == "" || hubName == "" || timestamp == "" {
		log.Printf("the event properties are missing device ID, hub name, or timestamp")
		return
	}
	var decoded IotEventMessage
	if err := json.Unmarshal(ev.Body, &decoded); err != nil {
		log.Printf("failed to decode event body: %v", err)
		return
	}

	if decoded.Properties.Reported.DecodedPayload.IsRFSensingTelemetry() {
		// Save the latest RF sensing payload in memory.
		// Later on it will be saved to Azure blob storage alongside environment sensing payload.
		latestRFSensingPayload[hubName+deviceID] = decoded.Properties.Reported.DecodedPayload
		log.Printf("saved RF sensing payload in map key: %q", hubName+deviceID)
		return
	}

	if decoded.Properties.Reported.DecodedPayload.IsEnvironmentSensingTelemetry() {
		// Wait for the latest RF sensing payload.
		rfPayload, exists := latestRFSensingPayload[hubName+deviceID]
		if !exists {
			log.Printf("waiting for RF sensing payload before saving to blob storage")
			return
		}
		newRecord, err := json.Marshal(flattenedRecord(ev.Properties, decoded.Properties.Reported.DecodedPayload, rfPayload))
		if err != nil {
			log.Panic(err)
			return
		}
		timeoutCtx, cancelFunc := context.WithTimeout(context.Background(), 10*time.Second)
		defer cancelFunc()
		_, err = appendBlobClient.AppendBlock(timeoutCtx, &BytesReadCloser{Reader: bytes.NewReader([]byte(string(newRecord) + "\n"))}, nil)
		if err != nil {
			log.Printf("failed to save the telemetry record: %+v", err)
			return
		}
		log.Printf("saved a telemetry record for hub %q, device %q", hubName, deviceID)
	}
}

func initBlobClient(containerClient *container.Client) *appendblob.Client {
	timeoutCtx, cancelFunc := context.WithTimeout(context.Background(), 60*time.Second)
	defer cancelFunc()
	blobName := time.Now().UTC().Format(time.DateOnly) + ".jsonl"
	appendBlobClient, err := appendblob.NewClientFromConnectionString(storageConnStr, storageContainer, blobName, nil)
	if err != nil {
		log.Panic(err)
	}
	// Create the blob if it does not exist yet.
	existingBlob := containerClient.NewBlobClient(blobName)
	_, err = existingBlob.GetProperties(timeoutCtx, nil)
	var respErr *azcore.ResponseError
	if errors.As(err, &respErr) {
		if respErr.StatusCode == http.StatusNotFound {
			if _, err := appendBlobClient.Create(timeoutCtx, nil); err != nil {
				log.Panic(err)
			}
			log.Printf("created new blob %q", blobName)
		} else {
			log.Panic(err)
		}
	}
	return appendBlobClient
}
