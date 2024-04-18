package main

import (
	"errors"
	"fmt"
	"io"
	"net/http"
	"strconv"
	"strings"
	"time"
)

const (
	SensorErratic      = iota
	SensorDisconnected = iota
	SensorOk           = iota
)

type SyncData struct {
	timestamp   int64
	tenantId    string
	version     string
	rssi        int // WiFi strength
	intervals   []int
	sensorState int
}

func parseSyncRequest(timestamp int64, requestStr string) (SyncData, error) {
	pairs := strings.Split(requestStr, ";")
	m := make(map[string]string)
	for _, pair := range pairs {
		kv := strings.Split(pair, "=")
		key := kv[0]
		value := kv[1]
		m[key] = value
	}

	syncData := SyncData{timestamp: timestamp}
	tenant, ok := m["tenantId"]
	if ok {
		syncData.tenantId = tenant
	} else {
		return SyncData{}, errors.New("missing tenantId")
	}
	rssiStr, ok := m["rssi"]
	if ok {
		rssi, err := strconv.Atoi(rssiStr)
		if err != nil {
			return SyncData{}, errors.New("rssi must be an integer")
		}
		syncData.rssi = rssi
	} else {
		return SyncData{}, errors.New("Missing rssi")
	}

	version, ok := m["version"]
	if ok {
		syncData.version = version
	} else {
		return SyncData{}, errors.New("missing version")
	}

	sensor, ok := m["sensor"]
	if ok {
		if sensor == "ok" {
			syncData.sensorState = SensorOk
		} else if sensor == "erratic" {
			syncData.sensorState = SensorErratic
		} else if sensor == "discon" {
			syncData.sensorState = SensorDisconnected
		}
	}

	intervalsStr, ok := m["values"]
	if ok {
		intervalsStrParts := strings.Split(intervalsStr, ",")
		if len(strings.TrimSpace(intervalsStr)) == 0 {
			intervalsStrParts = []string{}
		} else if strings.HasSuffix(intervalsStr, ",") {
			// Should always be the case - to keep code on esp simple
			intervalsStrParts = intervalsStrParts[:len(intervalsStrParts)-1]
		}
		for _, part := range intervalsStrParts {
			interval, err := strconv.Atoi(part)
			if err != nil {
				return SyncData{}, errors.New("interval must be an integer")
			}
			syncData.intervals = append(syncData.intervals, interval)
		}
	} else {
		return SyncData{}, errors.New("missing intervals")
	}

	return syncData, nil
}

func main() {
	const host = "192.168.1.12:8090"
	http.HandleFunc("/v1/sync", uploadSync)

	fmt.Printf("Running on http://%s\n", host)
	http.ListenAndServe(host, nil)
}

func uploadSync(w http.ResponseWriter, req *http.Request) {
	buf := new(strings.Builder)
	_, err := io.Copy(buf, req.Body)
	if err != nil {
		fmt.Fprintf(w, "ERROR\n")
		w.WriteHeader(http.StatusBadRequest)
	} else {
		fmt.Println(buf.String())
		syncData, err := parseSyncRequest(time.Now().UnixMilli(), buf.String())
		if err != nil {
			fmt.Printf("Failed to parse sync object: %s\n", err)
			w.WriteHeader(http.StatusBadRequest)
		} else {
			fmt.Printf("%+v\n", syncData)
		}
	}
}
